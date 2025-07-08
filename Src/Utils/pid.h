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

#ifndef PID_H_
#define PID_H_

#include <stdbool.h>

#define DEFAULT_PID_INTEGRATION_LIMIT 5000.0
#define DEFAULT_PID_OUTPUT_LIMIT      0.0

typedef struct{
	float kp;
	float ki;
	float kd;
}pid_t;

typedef struct{
	pid_t coefficient;
	float iLimit;
	float integral;			/* Integration */
	float dt;				/* Deltatime between measurements */

	float last_error;		/* Last Error */
	float last_measurement;	/* Last Measurement */

	float output_limit;

	bool enableDFilter;		/* Enable Low Pass Filter*/
	//lpf2pData  DFilter;	/* Derivative Low Pass Filter */
}pidHandle_t;

void  pidInit(pidHandle_t* handle);
void  pidReset(pidHandle_t* handle);
float pidUpdate(pidHandle_t* handle, float measurement, float desired);

#endif /* PID_H_ */
