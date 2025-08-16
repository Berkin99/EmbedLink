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
#include "sensor.h"

#define ESTIMATOR_INITIALIZE_TIMEOUT_MS    10000
#define ESTIMATOR_STABILIZE_MS			   5000

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

#define STATE_TYPECOUNT SENSE_TYPECOUNT

typedef union{
	struct 
	{
		xvec_t position;
		xvec_t rotation;
		xvec_t velocity;
		xvec_t acceleration;
		xvec_t attitude;
		xvec_t irotation;
		xvec_t ivelocity;
		xvec_t iacceleration;
		xvec_t iattitude;
		xvec_t magnetization;
		xvec_t pressure;
		xvec_t temperature;
	};
	xvec_t v[STATE_TYPECOUNT];
}state_t;

typedef struct{
	char*   name;
	int8_t  (*init)(void);
	int8_t  (*test)(void);
	int8_t  (*isReady)(void);
	void    (*originSet)(void);
}estimator_t;

void   estimatorInit (void);
void   estimatorTest (void);
int8_t estimatorIsReady(void);
void   estimatorReset(state_t* pState);
void   estimatorIterate(state_t* pState);
void   estimatorUpdate(state_t* base, const state_t* update);
void   estimatorStabilize(state_t* pState, uint32_t timeoutMs);
void   estimatorOriginSet(void);

#endif /* ESTIMATOR_H_ */
