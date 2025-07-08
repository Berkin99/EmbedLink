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
#include "systime.h"
#include "sysconfig.h"
#include "rtos.h"
#include "uart.h"
#include "estimator.h"


#ifdef ESTIMATOR_COMPLEMENTARY
#include "estimator_cf.h"
#endif
#ifdef ESTIMATOR_KALMAN
#include "estimator_kf.h"
#endif

#define ESTIMATOR_SET(EST) {          \
    .name = estimatorName##EST,       \
    .init = estimatorInit##EST,       \
    .test = estimatorTest##EST,       \
    .isReady = estimatorIsReady##EST, \
}

static estimator_t estimators[] = {
#ifdef ESTIMATOR_KALMAN
		ESTIMATOR_SET(KF),
#endif
#ifdef ESTIMATOR_COMPLEMENTARY
		ESTIMATOR_SET(CF),
#endif
};

static uint8_t isDefined = sizeof(estimators) / sizeof(estimator_t);
static estimator_t estimator;
static uint8_t isInit;

void estimatorInit (void){

	if(isDefined == 0) return; /* No estimator found */
	if(isInit) return;

	estimator = estimators[0]; /* Changable estimator for various estimators performance test */

	/* Wait Sensor Module */
	serialPrint("[>] Estimator waiting sensor & navigation...\n");
	uint32_t i = 0;
	while (++i < ESTIMATOR_INITIALIZE_TIMEOUT_MS){
		if (i % 1000 == 0) serialPrint(" *\n");
		delay(1);
		if(sensorIsReady()) break;
	}
	if (!(sensorIsReady())) serialPrint(" TIMEOUT\n");
	else serialPrint(" READY\n");

	/* Init Estimator Object */
	if (estimator.init() == OK) serialPrint("[+] Estimator %s init OK\n", estimator.name);
	else serialPrint("[-] Estimator %s init ERROR\n" , estimator.name);

	isInit = 1;
}

void estimatorTest(void){
	if(isDefined == 0) return;
	if(estimator.test() == OK) serialPrint("[+] Estimator %s test OK\n", estimator.name);
	else serialPrint("[-] Estimator %s test ERROR\n" , estimator.name);
}

int8_t estimatorIsReady(void){
	if(!isInit || isDefined == 0) return E_NOT_FOUND;
	return estimator.isReady();
}

void estimatorIterate(state_t* pState){

	/* Set all states to zero */
	for (uint8_t i = 0; i < SENSE_TYPECOUNT; i++) {
		pState->v[i] = xvnew(vzero(), vrepeat(999999.0f), 0U);
	}

	/* For each sense in the sense queue */
	sense_t z;
	while(sensorDequeue(&z, 0) == TRUE){
		pState->v[z.type] = xvcomb(pState->v[z.type], z.xvec);
	}
}

void estimatorStabilize(state_t* pState, uint32_t timeoutMs){

	serialPrint("[>] Estimator stabilizing\n");

	estimatorIterate(pState);

	while(timeoutMs > 0){
		if(timeoutMs % 1000 == 0) serialPrint(" *\n");
		state_t tstate;
		estimatorIterate(&tstate);

		for (uint8_t i = 0; i < STATE_TYPECOUNT; i++) {
			if(tstate.v[i].timestampMs == 0) continue;
			pState->v[i] = xvcomb(pState->v[i], tstate.v[i]);
		}

		delay(1);
		timeoutMs--;
	}

	serialPrint("READY\n");

	for (uint8_t i = 0; i < STATE_TYPECOUNT; i++) {
		serialPrint("[%d] %.3f, %.3f, %.3f\n", i, pState->v[i].x, pState->v[i].y, pState->v[i].z);
	}
}
