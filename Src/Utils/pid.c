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

#include <math.h>
#include "pid.h"
#include "num.h"

void pidInit(pidHandle_t* handle){
	handle->iLimit = DEFAULT_PID_INTEGRATION_LIMIT;
	handle->output_limit = DEFAULT_PID_OUTPUT_LIMIT;
	pidReset(handle);
}

void pidReset(pidHandle_t* handle){

	handle->integral = 0;
	handle->last_measurement = 0;
	handle->last_error = 0;
}


/**
 * IN: Handle -> Pid Handle
 * IN: Measurement -> Filtered Angular Velocity *deg/s [?,?]
 * IN: Desired -> Desired Angular Velocity (AngleError * K) deg/s [-900,900] K = 5
 * OUT: Pid Output -> Motor Power Difference [900/3]-> [300]
 */
float pidUpdate(pidHandle_t* handle, float measurement, float desired){

	float output = 0.0;

	/* Calculated P Value added to output */
	float error = desired - measurement;
	output += handle->coefficient.kp * error;

	/* Calculated I Value added to output */
	handle->integral += handle->coefficient.ki * (error * handle->dt); //1.6
	handle->integral = constrainFloat(handle->integral, -handle->iLimit, handle->iLimit);
	output += handle->integral;

	/* Calculated D Value added to output */
	float derivative = (error - handle->last_error) / handle->dt; /* Derivative */
	output += handle->coefficient.kd * derivative;  /* Calculated D Value added to output */
	/*TODO: ADD Low pass filter to Derivative*/

	handle->last_error = error;
	handle->last_measurement = measurement;

	return output;
}
