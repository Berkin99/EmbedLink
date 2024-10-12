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

#ifndef ESTIMATOR_H_
#define ESTIMATOR_H_

#include <stdint.h>
#include "kinematics.h"
#include "navigation.h"

#define ESTIMATOR_INITIALIZE_TIMEOUT_MS    10000
#define ESTIMATOR_STABILIZE_MS			   2000

/* ESTIMATOR TYPE */
#define ESTIMATOR_KALMAN
//#define ESTIMATOR_COMPLEMENTARY

#define RATE_1000_HZ 1000
#define RATE_500_HZ  500
#define RATE_250_HZ  250
#define RATE_100_HZ  100
#define RATE_50_HZ   50
#define RATE_25_HZ   25

#define ESTIMATOR_FREQ_HZ RATE_1000_HZ
#define RATE_DO_EXECUTE(RATE_HZ, TICK) ((TICK % (ESTIMATOR_FREQ_HZ / RATE_HZ)) == 0)

typedef struct{
	char*   name;
	int8_t  (*init)(void);
	int8_t  (*test)(void);
	int8_t  (*isReady)(void);
	const state_t* (*state)(void);
}estimator_t;

void estimatorInit (void);
void estimatorTest (void);
int8_t estimatorIsReady(void);

void estimatorStabilize(state_t* pState, uint32_t tim);
void estimatorUpdate(state_t* pState, uint8_t* pChecklist);

int8_t estimatorEnqueue(const measurement_t* pMeasurement, int8_t isISR);
int8_t estimatorDequeue(measurement_t* pMeasurement, uint32_t portDelay);

#endif /* ESTIMATOR_H_ */
