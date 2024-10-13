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

#ifdef  ESTIMATOR_COMPLEMENTARY

#include "systime.h"
#include "math3d.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "static_mem.h"
#include "estimator_cf.h"
#include "serial.h"
#include "geoconfig.h"
#include "navigation.h"
#include "kinematics.h"
#include "sensor.h"
#include "madgwick.h"
#include "num.h"


#define ESTIMATOR_RATE			RATE_1000_HZ
#define MADGWICK_UPDATE_RATE	RATE_1000_HZ
#define COMPASS_UPDATE_RATE	    RATE_100_HZ
#define BAROMETER_UPDATE_RATE	RATE_100_HZ

static state_t    state;
static navstate_t navstate;
static navstate_t origin;

static uint8_t isInit;
static uint8_t isReady;

void _estimatorOriginSetCF(void);
void _estimatorUpdateCF(estimatorTick_t tick);
void _estimatorMadgwickCF(void);
void _estimatorCompassCF(void);
void _estimatorBarometerCF(void);

void estimatorTaskCF(void* argv);
STATIC_MEM_TASK_ALLOC(ESTIMATOR_CF, ESTIMATOR_TASK_STACK, ESTIMATOR_TASK_PRI);

int8_t estimatorInitCF (void){

	if(isInit) return SYS_E_OVERWRITE;

	STATIC_MEM_TASK_CREATE(ESTIMATOR_CF, estimatorTaskCF, NULL);
	isInit = 1;

	while(!isReady) delay(1);

	return SYS_OK;
}

int8_t estimatorTestCF (void){
    if(isInit) return SYS_OK;
    return SYS_E_NOT_FOUND;
}

const state_t* estimatorStateCF(void){
	return &state;
}

int8_t estimatorIsReadyCF (void){
	return isReady;
}

void estimatorTaskCF(void* argv){

	estimatorTick_t xEstimatorTime = 0;

	estimatorStabilize(&state, ESTIMATOR_STABILIZE_MS);
	_estimatorOriginSetCF();

	TickType_t xLastWakeTime = xTaskGetTickCount();
	isReady = 1;

	while(1){
		_estimatorUpdateCF(xEstimatorTime);
		kinematicsStateUpdate(&state);
		xEstimatorTime++;
		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1000 / ESTIMATOR_RATE));
	}
}

void _estimatorOriginSetCF(void){
	/* Set Current altitude as zero */
	navigation_t nav;
	nav.type = NAV_ALTITUDE;
	nav.altitude = navigationPressureToAltitude(state.pressure.x);
	navigationOriginSet(&nav);

	navstate.altitude = nav.altitude;

	nav.type = NAV_COMPASS;
	_estimatorCompassCF();
	nav.compass = navstate.compass;
	navigationOriginSet(&nav);

	_estimatorMadgwickCF();
	madgwickSetBaseZAcc(madgwickGetAccZ(state.iacceleration.x, state.iacceleration.y, state.iacceleration.z));

	origin = navigationOrigin();
}

void _estimatorUpdateCF(estimatorTick_t tick){
    /* Estimator responsable from kinematics update */
    uint8_t checklist[STATE_TYPECOUNT];
    estimatorUpdate(&state, checklist);

    if(RATE_DO_EXECUTE(MADGWICK_UPDATE_RATE, tick)){  	_estimatorMadgwickCF();  }
    if(checklist[STATE_PRESSURE] > 0) _estimatorBarometerCF();

//	if(RATE_DO_EXECUTE(COMPASS_UPDATE_RATE, tick)){   	_estimatorCompassCF();   }
}

void _estimatorMadgwickCF(void){
	madgwickUpdateQ(state.iattitude.x, state.iattitude.y, state.iattitude.z,
	state.iacceleration.x, state.iacceleration.y, state.iacceleration.z, (1.0f / MADGWICK_UPDATE_RATE));

	quat_t qt;
	madgwickGetQuaternion(&qt.x, &qt.y, &qt.z, &qt.w);

	madgwickGetEulerRPY(&state.rotation.y, &state.rotation.x, &state.rotation.z);

	state.acceleration.z = (state.acceleration.z + madgwickGetAccZWithoutGravity(state.iacceleration.x, state.iacceleration.y, state.iacceleration.z)) / 2.0f;
	/* World Velocity Z */
	state.velocity.z += deadbandFloat(state.acceleration.z, 0.04f) *  (1.0f / MADGWICK_UPDATE_RATE); /* m/s'2 to m/s */
	state.position.z += state.velocity.z * (1.0f / MADGWICK_UPDATE_RATE);
}

void _estimatorCompassCF(void){

	#define ALPHA_COMPASS_Z 0.02f
	/* Magnetometer may be tilted, this calculation fixes values to the world frame */
	float cps[2];
	cps[0] = state.magnetization.y * cosf( -state.rotation.x * DEG2RAD)
		   - state.magnetization.x * sinf(  state.rotation.y * DEG2RAD) * sinf(-state.rotation.x * DEG2RAD)
		   + state.magnetization.z * cosf(  state.rotation.y * DEG2RAD) * sinf(-state.rotation.x * DEG2RAD);

	cps[1] = - state.magnetization.x * cosf( state.rotation.y * DEG2RAD)
			 - state.magnetization.z * sinf( state.rotation.y * DEG2RAD);

	/* 2D Vector to rotation 0 is NORTH [-180 to 180]*/
	navstate.compass = (atan2f(cps[1], cps[0]) * RAD2DEG ) + MAGNETIC_DECLINATION;
	if(navstate.compass >  180) navstate.compass -= 360.0f;
	if(navstate.compass < -180) navstate.compass += 360.0f;

	float zrot = navstate.compass;
	state.rotation.z = state.rotation.z * (1.0f - ALPHA_COMPASS_Z) + zrot * ALPHA_COMPASS_Z;
}

void _estimatorBarometerCF(void){

	#define ALPHA_BARFAST 0.2f
	#define ALPHA_BARSLOW 0.995f
    static float barfast;
    static float lastbar;
    static float barvel;

    static uint32_t lasttime;
	static float _dt; /*deltatime as seconds*/

	_dt = ((float)(micros() - lasttime)) / 1000000; /*microseconds to seconds*/
	lasttime = micros();

	///////////////////////////////////////////////////////////////////////////////////////////

	/* Bar Fast */
	barfast = (ALPHA_BARFAST) * barfast +
	(1.0f - ALPHA_BARFAST) * navigationPressureToAltitude(state.pressure.x);

	/* Bar Slow */
	navstate.altitude = (ALPHA_BARSLOW) * navstate.altitude +
    (1.0f - ALPHA_BARSLOW) * navigationPressureToAltitude(state.pressure.x);

	/* Bar Compensation */
	float dAtt = fabsf(navstate.altitude - barfast); /* Difference between barslow and barfast */

	if(dAtt > 0.1f){
		dAtt -= 0.1f;
		dAtt += 1;
		float alpha = dAtt * dAtt;
		alpha -= 1;
		alpha = constrainFloat(alpha, 0.0f, 5.0f);
		alpha /= 5.0f;
		navstate.altitude = (1.0f - alpha) * navstate.altitude +
		(alpha) * barfast;
	}

	/* Velocity Compenstaion */
	if(lastbar != 0) barvel = (navstate.altitude - lastbar) / _dt;
	lastbar = navstate.altitude;

	state.velocity.z = state.velocity.z * 0.995f + barvel * 0.005f; /* (ALPHAVEL + COMPENSATION VALUE) */

	/* Position Correction */
	state.position.z = state.position.z*0.9 + 0.1*(navstate.altitude - origin.altitude);
}

#endif
