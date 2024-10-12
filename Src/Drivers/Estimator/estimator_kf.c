/*
 *       ______          __             ____    _       __
 *      / ____/___ ___  / /_  ___  ____/ / /   (_)___  / /__
 *     / __/ / __ `__ \/ __ \/ _ \/ __  / /   / / __ \/ //_/
 *    / /___/ / / / / / /_/ /  __/ /_/ / /___/ / / / / ,<
 *   /_____/_/ /_/ /_/_.___/\___/\__,_/_____/_/_/ /_/_/|_|
 *
 *  EmbedLink Firmware
 *  Copyright (c) 2024 Yeniay RD, All rights reserved.
 *  _________________________________________________________
 *
 *  EmbedLink Firmware is free software: you can redistribute
 *  it and/or  modify it under  the  terms of the  GNU Lesser
 *  General Public License as  published by the Free Software
 *  Foundation,  either version 3 of the License, or (at your
 *  option) any later version.
 *
 *  EmbedLink  Firmware is  distributed  in the  hope that it
 *  will be useful, but  WITHOUT  ANY  WARRANTY; without even
 *  the implied warranty of MERCHANTABILITY or FITNESS FOR A
 *  PARTICULAR PURPOSE.  See  the GNU  Lesser  General Public
 *  License for more details.
 *
 *  You should have received a copy of the GNU Lesser General
 *  Public License along with EmbedLink Firmware. If not, see
 *  <http://www.gnu.org/licenses/>.
 *
 */

#include "sysconfig.h"
#include "sysdebug.h"
#include "systime.h"
#include "math3d.h"
#include "rtos.h"
#include "estimator_kf.h"
#include "estimator.h"
#include "uart.h"
#include "geoconfig.h"
#include "navigation.h"
#include "kinematics.h"
#include "sensor.h"
#include "madgwick.h"
#include "num.h"
#include "kalman1D.h"

#ifdef ESTIMATOR_KALMAN

#define ESTIMATOR_RATE			RATE_1000_HZ
#define MADGWICK_UPDATE_RATE	RATE_1000_HZ
#define COMPASS_UPDATE_RATE	    RATE_100_HZ
#define KF_UPDATE_RATE			RATE_250_HZ
#define KF_POS_UPDATE_RATE	    RATE_1000_HZ

static state_t state;
static uint8_t checklist[STATE_TYPECOUNT];

static uint8_t 	  isInit;
static uint8_t 	  isReady;
static kalman_t   hKalman[3];

void _estimatorMadgwickKF(void);
void _estimatorCompassKF(void);
void _estimatorHeightKF(void);
void _estimatorPositionKF(void);
void _estimatorOriginSetKF(void);
void _estimatorUpdateKF(uint32_t tick);

void estimatorTaskKF(void* argv);
taskAllocateStatic(ESTIMATOR_KF, ESTIMATOR_TASK_STACK, ESTIMATOR_TASK_PRI);

int8_t estimatorInitKF (void){
	if(isInit) return SYS_E_OVERWRITE;
	taskCreateStatic(ESTIMATOR_KF, estimatorTaskKF, NULL);
	isInit = 1;
	while(!isReady) delay(1);
	return OK;
}

int8_t estimatorTestKF (void){
	return OK;
}

void estimatorTaskKF(void* argv){

	kalmanInit(&hKalman[0], (1.0f / KF_POS_UPDATE_RATE),  0.2f,  0.8f);   /* Navigation X */
	kalmanInit(&hKalman[1], (1.0f / KF_POS_UPDATE_RATE),  0.2f,  0.8f);   /* Navigation Y */
	kalmanInit(&hKalman[2], (1.0f / KF_UPDATE_RATE),      0.3f,  0.1f);   /*  Pressure Z  */

	estimatorStabilize(&state, ESTIMATOR_STABILIZE_MS);
	kinematicsStateUpdate(&state, checklist);
	_estimatorOriginSetKF();

	uint32_t estimatorTime = 0;
	uint32_t lastWakeTime = taskGetTickCount();
	isReady = 1;

	while(1){
		_estimatorUpdateKF(estimatorTime);
		estimatorTime++;
		taskDelayUntil(&lastWakeTime, (1000 / ESTIMATOR_RATE));
	}
}

void _estimatorUpdateKF(estimatorTick_t tick){
    /* Estimator responsable from kinematics update */
    estimatorUpdate      (&state, checklist);
	kinematicsStateUpdate(&state, checklist);

    if(RATE_DO_EXECUTE(MADGWICK_UPDATE_RATE, tick)) _estimatorMadgwickKF();
    if(RATE_DO_EXECUTE(COMPASS_UPDATE_RATE,  tick)) _estimatorCompassKF();
	if(RATE_DO_EXECUTE(KF_UPDATE_RATE,       tick)) _estimatorHeightKF();
	if(RATE_DO_EXECUTE(KF_POS_UPDATE_RATE,   tick)) _estimatorPositionKF();
}

void _estimatorMadgwickKF(void){

	kinv_t iatt;
	kinv_t iacc;
	kinv_t rotation     = kinzero();
	kinv_t acceleration = kinzero();

	/* Rotation Estimate */
	if(kinematicsVector(STATE_INTERNAL_ATTITUDE, &iatt) <= 0) return;
	if(kinematicsVector(STATE_INTERNAL_ACCELERATION, &iacc) <= 0) return;

	madgwickUpdateQ(iatt.x, iatt.y, iatt.z, iacc.x, iacc.y, iacc.z, (1.0f / MADGWICK_UPDATE_RATE));
	madgwickGetEulerRPY(&rotation.y, &rotation.x, &rotation.z);
	rotation.stdDev = iatt.stdDev;

	/* Acceleration Estimate */
	float fixrot; if(!navigationCompass(&fixrot)) fixrot = rotation.z; /* If North is not defined World frame is start XYZ */

	fixrot = rotation.z; /* NOCOMPASS POSITIONING */

	acceleration.vector = kinematicsRotateFrame(iacc.vector, mkvec(rotation.x, rotation.y, fixrot));

	acceleration.stdDev = iacc.stdDev;
	acceleration.z      = madgwickGetAccZWithoutGravity(iacc.x, iacc.y, iacc.z);

	kinematicsAppend(STATE_ROTATION,     rotation);
	kinematicsAppend(STATE_ACCELERATION, acceleration); /* Acceleration Error */
}

void _estimatorCompassKF(void){
	static float compass;
	kinv_t mag;
	kinv_t rot;
	navigation_t nav;

	/* Compass Estimate */
	if(kinematicsVector(STATE_MAGNETIZATION, &mag) <= 0) return;
	if(kinematicsVector(STATE_ROTATION, &rot)      <= 0) return;

	vec_t vcmp = kinematicsRotateFrame(mag.vector, mkvec(rot.x, rot.y, 0));
	vcmp.z = (atan2f(vcmp.x, vcmp.y) * RAD2DEG);

	float dif = courseDeviation(vcmp.z, compass);
	compass = courseDeviation(compass + (dif * 0.018), 0);

	nav.type    = NAV_COMPASS;
	nav.compass = courseDeviation(compass - MAGNETIC_DECLINATION, 0);
	navigationAppend(&nav);
}

void _estimatorHeightKF(void){
	/* Kalman Filter */
	kinv_t pres;
	kinv_t acc;
	kinv_t velocity = kinzero();
	kinv_t position = kinzero();
	/* Height Estimate */
	if(kinematicsVector(STATE_PRESSURE, &pres) <= 0 || kinematicsVector(STATE_ACCELERATION, &acc) <= 0) return;

	vec_t zn;
	zn.z = navigationPressureToAltitude(pres.value) - navigationOrigin().altitude;
	kalmanIterate(&hKalman[2], zn.z, acc.z);
	position.z   = hKalman[2].Xn.mx[0][0];
	velocity.z   = hKalman[2].Xn.mx[1][0];

	position.stdDev.z = 1;
	velocity.stdDev.z = acc.stdDev.z;

	kinematicsAppend(STATE_POSITION, position);
	kinematicsAppend(STATE_VELOCITY, velocity);

	/* Altitude Append */
	navigation_t altitude;
	altitude.type = NAV_ALTITUDE;
	altitude.altitude = navigationOrigin().altitude + position.z;
	navigationAppend(&altitude);
}

void _estimatorPositionKF(void){
	static vec_t posDif;
	static vec_t posLast;
	static int    posCounter;

	kinv_t     acc;
	location_t loc;
	compass_t  cmp;
	kinv_t velocity = kinzero();
	kinv_t position = kinzero();

	if(navigationLocation(&loc) <= 0 || navigationCompass(&cmp) <= 0) return;
	if(kinematicsVector(STATE_ACCELERATION, &acc) <= 0) return;

	kinv_t pos;
	navigationLocationPos(&pos);

	posCounter++;
	pos.vector = vadd(pos.vector, posDif);
	if(posCounter >= 200){
		posDif = vdiv(vsub(pos.vector, posLast), 200);
		posLast = pos.vector;
	}

	kalmanIterate(&hKalman[0], pos.x, acc.x);
	position.x = hKalman[0].Xn.mx[0][0];
	velocity.x = hKalman[0].Xn.mx[1][0];

	kalmanIterate(&hKalman[1], pos.y, acc.y);
	position.y = hKalman[1].Xn.mx[0][0];
	velocity.y = hKalman[1].Xn.mx[1][0];

	velocity.stdDev = mkvec(acc.stdDev.x, acc.stdDev.y, 0);
	position.stdDev = mkvec(1, 1, 0);

	kinematicsAppend(STATE_POSITION, position);
	kinematicsAppend(STATE_VELOCITY, velocity);
}

void _estimatorOriginSetKF(void){
	/* Set Current altitude as zero */
	navigation_t nav;
	nav.type = NAV_ALTITUDE;
	nav.altitude = navigationPressureToAltitude(state.pressure.value);
	navigationOriginSet(&nav);

	nav.type = NAV_COMPASS;
	_estimatorCompassKF();
	navigationCompass(&nav.compass);
	navigationOriginSet(&nav);

	madgwickUpdateQ(state.iattitude.x, state.iacceleration.y, state.iacceleration.z,
	state.iacceleration.x, state.iacceleration.y, state.iacceleration.z, (1.0f / MADGWICK_UPDATE_RATE));
	madgwickSetBaseZAcc(madgwickGetAccZ(state.iacceleration.x, state.iacceleration.y, state.iacceleration.z));
}

int8_t estimatorIsReadyKF (void){
	return isReady;
}

const state_t* estimatorStateKF(void){
	return &state;
}

#endif
