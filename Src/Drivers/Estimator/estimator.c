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
#include "navigator.h"
#include "sensor.h"
#include "estimator.h"

#ifdef ESTIMATOR_COMPLEMENTARY
#include "estimator_cf.h"
#endif
#ifdef ESTIMATOR_KALMAN
#include "estimator_kf.h"
#endif

#define MEASUREMENTS_QUEUE_SIZE (20)

#define ESTIMATOR_SET(EST) {          \
    .name = estimatorName##EST,       \
    .init = estimatorInit##EST,       \
    .test = estimatorTest##EST,       \
    .isReady = estimatorIsReady##EST, \
	.state = estimatorState##EST,     \
}

static queue_t qmeasure;
queueAllocateStatic(qmeasure, MEASUREMENTS_QUEUE_SIZE, sizeof(measurement_t))

static estimator_t estimators[] = {
#ifdef ESTIMATOR_COMPLEMENTARY
		ESTIMATOR_SET(CF),
#endif
#ifdef ESTIMATOR_KALMAN
		ESTIMATOR_SET(KF),
#endif
};

static uint8_t isDefined = sizeof(estimators) / sizeof(estimator_t);
static estimator_t estimator;
static uint8_t isInit;
static uint8_t qready;

void estimatorInit (void){

	if(isDefined == 0) return;
	estimator = estimators[0]; /* Changable estimator for various estimators performance test */

	qmeasure = queueCreateStatic(qmeasure);
	qready = 1;

	uint32_t i = 0;

	serialPrint("[>] Estimator waiting sensor & navigation...\n");

	while (++i < ESTIMATOR_INITIALIZE_TIMEOUT_MS){
		if (i % 1000 == 0) serialPrint(" *\n");
		delay(1);
		if(sensorIsReady()) break;
	}

	if (!(sensorIsReady())) serialPrint(" TIMEOUT\n");
	else serialPrint(" READY\n");

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

void estimatorStabilize(state_t* pState, uint32_t tim){

	serialPrint("[>] Estimator stabilizing...\n");

	float n[STATE_TYPECOUNT] = {0}; /* sum iteration */
	uint8_t checklist[STATE_TYPECOUNT];
	estimatorUpdate(pState, checklist);

	state_t state;
	while(tim > 0){
		if(tim % 1000 == 0) serialPrint(" *\n");
		estimatorUpdate(&state, checklist);

		for (uint8_t i = 0; i < STATE_TYPECOUNT; i++) {
			if(checklist[i] == 0) continue;
			n[i]++;
			pState->kinv[i].vector = vadd(vscl(pState->kinv[i].vector, ((n[i] - 1) / n[i])), vscl(state.kinv[i].vector, (1.0f / n[i])));
		}

		delay(1);
		tim--;
	}

	kinematicsStateUpdate(pState, checklist);

	serialPrint(" READY\n");

	for (uint8_t i = 0; i < STATE_TYPECOUNT; ++i) {
		serialPrint("[%d] %.3f, %.3f, %.3f\n", i, pState->kinv[i].x, pState->kinv[i].y, pState->kinv[i].z);
	}
}

void estimatorUpdate(state_t* pState, uint8_t* pChecklist){

	state_t tState;
	measurement_t z;
	vec_t wsum[STATE_TYPECOUNT];

	/* Set all states to zero */
	for (uint8_t i = 0; i < STATE_TYPECOUNT; i++) {
		pChecklist[i] = 0;
		tState.kinv[i].vector = vzero();
		tState.kinv[i].stdDev = vzero();
		wsum[i]               = vzero();
	}

	/* For each measurement in the measurement queue */
	while(estimatorDequeue(&z, 0) == TRUE){
		/* For each axis */
		pChecklist[z.type] = 1; /* Sign the related type */

		for (uint8_t i = 0; i < 3; i++) {
			if(z.kinv.stdDev.axis[i] == 0) continue; /* Standard deviation error */
			float weight = 1.0f / (z.kinv.stdDev.axis[i] * z.kinv.stdDev.axis[i]); /* Weight of the measurement */
			wsum[z.type].axis[i] += weight;
			tState.kinv[z.type].vector.axis[i] += z.kinv.vector.axis[i] * weight;
		}
	}

	/* For each state type */
	for (uint8_t i = 0; i < STATE_TYPECOUNT; ++i) {
		/* For each axis */
		for (uint8_t j = 0; j < 3; j++) {
			if(wsum[i].axis[j] == 0) continue; /* Protect from zero division */

			pState->kinv[i].vector.axis[j] = tState.kinv[i].vector.axis[j] / wsum[i].axis[j];
			pState->kinv[i].stdDev.axis[j] = sqrtf(1.0f / wsum[i].axis[j]);
		}
	}
}

int8_t estimatorEnqueue(const measurement_t* pMeasurement, int8_t isISR){
	if(!qready) return ERROR;
	int8_t result;
    if (isISR) result = queueSendISR(qmeasure, pMeasurement);
    else result = queueSend(qmeasure, pMeasurement, 0);
    return result;
}

int8_t estimatorDequeue(measurement_t* pMeasurement, uint32_t portDelay){
	if(!qready) return ERROR;

	if (queueReceive(qmeasure, pMeasurement, portDelay) == RTOS_TRUE) return TRUE;

	return FALSE;
}

