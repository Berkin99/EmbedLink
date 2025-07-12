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

#include <sysdefs.h>
#include "sysconfig.h"
#include "estimator.h"

#ifdef ESTIMATOR_KALMAN

#include "systime.h"
#include "xmath3d.h"
#include "xmath.h"
#include "rtos.h"
#include "estimator_kf.h"
#include "uart.h"
#include "geoconfig.h"
#include "navigation.h"
#include "kinematics.h"
#include "sensor.h"
#include "madgwick.h"
#include "kalman1D.h"
#include "uart.h"

#define ESTIMATOR_RATE			RATE_1000_HZ
#define ESTIMATOR_TIMEOUT_MS	(1000)
#define MADGWICK_UPDATE_RATE	RATE_1000_HZ
#define COMPASS_UPDATE_RATE	    RATE_100_HZ
#define KF_UPDATE_RATE			RATE_250_HZ
#define KF_POS_UPDATE_RATE	    RATE_1000_HZ

static state_t 	  state;
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
	if(isInit) return E_OVERWRITE;

	kalmanInit(&hKalman[0], (1.0f / KF_POS_UPDATE_RATE),  0.2f,  0.8f);   /* Navigation X */
	kalmanInit(&hKalman[1], (1.0f / KF_POS_UPDATE_RATE),  0.2f,  0.8f);   /* Navigation Y */
	kalmanInit(&hKalman[2], (1.0f / KF_UPDATE_RATE),      0.3f,  0.1f);   /*  Pressure Z  */

	taskCreateStatic(ESTIMATOR_KF, estimatorTaskKF, NULL);
	isInit = 1;
	while(!isReady) delay(1);
	return OK;
}

int8_t estimatorTestKF (void){
	return OK;
}

void estimatorTaskKF(void* argv){

	estimatorStabilize(&state, ESTIMATOR_STABILIZE_MS);

	xkinematicsSet(KINV_POSITION, state.position);
	xkinematicsSet(KINV_ROTATION, state.rotation);
	xkinematicsSet(KINV_VELOCITY, state.velocity);
	xkinematicsSet(KINV_ACCELERATION, state.acceleration);
	xkinematicsSet(KINV_ATTITUDE, state.attitude);
	xkinematicsSet(KINV_IROTATION, state.irotation);
	xkinematicsSet(KINV_IVELOCITY, state.ivelocity);
	xkinematicsSet(KINV_IACCELERATION, state.iacceleration);
	xkinematicsSet(KINV_IATTITUDE, state.iattitude);
	
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

void _estimatorUpdateKF(uint32_t tick){
    /* Estimator responsible for updating the kinematics */
	state_t tstate;
	estimatorReset(&tstate);
    estimatorIterate(&tstate);
	estimatorUpdate(&state, &tstate);

	/* TODO: Advanced iacceleration & iattitude calculation */
	xkinematicsSet(KINV_IACCELERATION, state.iacceleration);
	xkinematicsSet(KINV_IATTITUDE, state.iattitude);
	
    if(RATE_DO_EXECUTE(MADGWICK_UPDATE_RATE, tick))   _estimatorMadgwickKF();
    //if(RATE_DO_EXECUTE(COMPASS_UPDATE_RATE,  tick)) _estimatorCompassKF();
	if(RATE_DO_EXECUTE(KF_UPDATE_RATE,       tick)) _estimatorHeightKF();
	//if(RATE_DO_EXECUTE(KF_POS_UPDATE_RATE,   tick)) _estimatorPositionKF();
}

void _estimatorMadgwickKF(void){

	if(!xkinematicsIsValid(KINV_IACCELERATION, ESTIMATOR_TIMEOUT_MS)) return;
	if(!xkinematicsIsValid(KINV_IATTITUDE, ESTIMATOR_TIMEOUT_MS)) return;
	
	xvec_t iacc = xkinematicsGet(KINV_IACCELERATION);
	xvec_t iatt = xkinematicsGet(KINV_IATTITUDE);
	xvec_t acceleration = xvzero();
	xvec_t rotation = xvzero();

	/* Rotation Estimate */
	madgwickUpdateQ(iatt.x, iatt.y, iatt.z, iacc.x, iacc.y, iacc.z, (1.0f / MADGWICK_UPDATE_RATE));
	madgwickGetEulerRPY(&rotation.y, &rotation.x, &rotation.z);
	
	/* TODO: Rotation stddev extrapolation */
	rotation.stdDev = iatt.stdDev;
	rotation.timestampMs = iatt.timestampMs;

	float fixrot; 
	fixrot = rotation.z; /* NOCOMPASS POSITIONING */

	/* Acceleration Estimate */
	acceleration.v      	 = kinematicsRotateFrame(iacc.v, vnew(rotation.x, rotation.y, fixrot));
	acceleration.stdDev 	 = iacc.stdDev;
	acceleration.timestampMs = iacc.timestampMs;
	acceleration.z           = madgwickGetAccZWithoutGravity(iacc.x, iacc.y, iacc.z);

	xkinematicsSet(KINV_ROTATION, rotation);
	xkinematicsSet(KINV_ACCELERATION, acceleration);
}

void _estimatorHeightKF(void){
	/* Kalman Filter */
	xvec_t acc;
	xvec_t velocity = xvzero();
	xvec_t position = xvzero();

	/* Height Estimate */
	if(!xvtime(&state.pressure, ESTIMATOR_TIMEOUT_MS)) return;
	if(!xkinematicsIsValid(KINV_ACCELERATION, ESTIMATOR_TIMEOUT_MS)) return;
	acc = xkinematicsState()->acceleration;

	vec_t zn;
	zn.z = navigationPressureToAltitude(state.pressure.x) - xnavigationOrigin()->altitude.v;
	kalmanIterate(&hKalman[2], zn.z, acc.z);
	position.z   = hKalman[2].Xn.mx[0][0];
	velocity.z   = hKalman[2].Xn.mx[1][0];

	position.stdDev.z = 1.0f;
	position.timestampMs = millis();
	velocity.stdDev.z = acc.stdDev.z;
	velocity.timestampMs = millis();

	xkinematicsSet(KINV_POSITION, position);
	xkinematicsSet(KINV_VELOCITY, velocity);

	/* Altitude Append */
	altitude_t navx;
	navx.v = xnavigationOrigin()->altitude.v + position.z;
	navx.timestampMs = millis();

	xnavigationSetAltitude(navx);
}

// void _estimatorCompassKF(void){
// 	static float compass;
// 	xvec_t mag;
// 	xvec_t rot;
// 	navigation_t nav;

// 	/* Compass Estimate */

// 	if(!xvtime(&state.magnetization, 1000)) return;
// 	if(!xvtime(&state.rotation, 1000)) return;
	
// 	vec_t vcmp = kinematicsRotateFrame(mag.v, vnew(rot.x, rot.y, 0));
// 	vcmp.z = (atan2f(vcmp.x, vcmp.y) * RAD2DEG);

// 	float dif = cycdiff32(vcmp.z, compass);
// 	compass = cycdiff32(compass + (dif * 0.018), 0);

// 	nav.type    = NAV_COMPASS;
// 	nav.compass.v = cycdiff32(compass - MAGNETIC_DECLINATION, 0);
// 	navigationAppend(&nav);
// }

// void _estimatorPositionKF(void){
// 	static vec_t posDif;
// 	static vec_t posLast;
// 	static int    posCounter;
	
// 	kinv_t     acc;
// 	location_t loc;
// 	compass_t  cmp;
// 	kinv_t velocity = kinzero();
// 	kinv_t position = kinzero();

// 	if(navigationLocation(&loc) <= 0 || navigationCompass(&cmp) <= 0) return;
// 	if(kinematicsVector(STATE_ACCELERATION, &acc) <= 0) return;

// 	kinv_t pos;
// 	navigationLocationPos(&pos);

// 	posCounter++;
// 	pos.vector = vadd(pos.vector, posDif);
// 	if(posCounter >= 200){
// 		posDif = vdiv(vsub(pos.vector, posLast), 200);
// 		posLast = pos.vector;
// 	}

// 	kalmanIterate(&hKalman[0], pos.x, acc.x);
// 	position.x = hKalman[0].Xn.mx[0][0];
// 	velocity.x = hKalman[0].Xn.mx[1][0];

// 	kalmanIterate(&hKalman[1], pos.y, acc.y);
// 	position.y = hKalman[1].Xn.mx[0][0];
// 	velocity.y = hKalman[1].Xn.mx[1][0];

// 	velocity.stdDev = mkvec(acc.stdDev.x, acc.stdDev.y, 0);
// 	position.stdDev = mkvec(1, 1, 0);

// 	kinematicsAppend(STATE_POSITION, position);
// 	kinematicsAppend(STATE_VELOCITY, velocity);
// }

void _estimatorOriginSetKF(void){
	/* Altitude Origin */
	xf32_t altitude;
	altitude.v = navigationPressureToAltitude(state.pressure.x);
	altitude.timestampMs = millis();
	altitude.stdDev = state.pressure.stdDev.x;
	
	xnavigationOrigin()->altitude = altitude;
	/* Compass Origin */
	//_estimatorCompassKF();
	//xnavigationOrigin()->compass = xnavigationState()->compass;

	/* Z Accel Reference */
	madgwickUpdateQ(state.iattitude.x, state.iacceleration.y, state.iacceleration.z,
	state.iacceleration.x, state.iacceleration.y, state.iacceleration.z, (1.0f / MADGWICK_UPDATE_RATE));
	madgwickSetBaseZAcc(madgwickGetAccZ(state.iacceleration.x, state.iacceleration.y, state.iacceleration.z));
}

int8_t estimatorIsReadyKF (void){
	return isReady;
}

#endif
