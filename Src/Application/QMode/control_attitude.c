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

#include "xmath.h"
#include "xmath3d.h"
#include "kinematics.h"
#include "quadcopter.h"
#include "pid.h"
#include "control_attitude.h"

/* Attitude PID */
#define POWER_MAX			800		//[0,1000] us
#define ANGLE_MAX			30		//Degrees
#define ANGLERATE_MAX		160		//Deg/s
#define PID_ROLLPITCH 		{1.1,  0.01,  17.0}
#define PID_ROLLPITCH_MAX 	300
#define PID_YAW		  		{3.0,	0.001,	0.8}
#define PID_YAW_MAX			300

static pidHandle_t hpidAtt	[3];
static pid_t pidAtt 		[3] = {PID_ROLLPITCH, PID_ROLLPITCH, PID_YAW};
static int   pidAttLimit	[3] = {PID_ROLLPITCH_MAX, PID_ROLLPITCH_MAX, PID_YAW_MAX};

void controlInitATTITUDE(void){
	/* Attitude PID Initialize */
	for(uint8_t i = 0; i < 3; i++){
		pidInit(&hpidAtt[i]);
		hpidAtt[i].coefficient = pidAtt[i];
		hpidAtt[i].iLimit = 30;
		hpidAtt[i].dt = 1;
	}
}

/* IN1 : CMD Power [0, 1]
 * IN2 : CMD Range = Target in [-1, 1]
 * OUT : Motor Powers [FR, RR, RL, FL] [0, 1]
 * [!] Run this Function with fixed 250 Hz rate
 * */
quadmotor_t controlTaskATTITUDE (float cpow, vec_t crange){

	static float targetAtt [3];
	static float att       [3];
	static float pidOut	   [3];
	quadmotor_t  mout;

	/* Pid Calculation for xyz axis */
	for (uint8_t i = 0; i < 3; i++){
		/* Target Angular Rate Calculation */ /* Angle [-30,30] , Range [-1, 1] -> Target Gyr Calculation [-320,320] */
		targetAtt[i]  = (crange.axis[i]) - (xkinematicsState()->rotation.axis[i] / ANGLE_MAX);
		targetAtt[i] *= ANGLERATE_MAX;

		/* PID Handle Update */
		hpidAtt[i].dt = 1;
		att[i] = (xkinematicsState()->iattitude.axis[i] * 0.2) + (att[i] * 0.8);
		pidOut[i] = pidUpdate(&hpidAtt[i], att[i], targetAtt[i]);
		pidOut[i] = clampf32(pidOut[i] , -pidAttLimit[i], pidAttLimit[i]);
	}

	/* Motor Power Calculation */
	cpow   = clampf32(cpow, 0, 1);
	mout.mFR  = cpow * POWER_MAX + pidOut[0] - pidOut[1] - pidOut[2]; // FR
	mout.mRR  = cpow * POWER_MAX - pidOut[0] - pidOut[1] + pidOut[2]; // RR
	mout.mRL  = cpow * POWER_MAX - pidOut[0] + pidOut[1] - pidOut[2]; // RL
	mout.mFL  = cpow * POWER_MAX + pidOut[0] + pidOut[1] + pidOut[2]; // FL

	for (uint8_t i = 0; i < 4; i++) {
		mout.m[i] = clampf32(mout.m[i], 60, 1000); /* Keep Motors Running */
		mout.m[i] /= 1000.0f;
	}
	return mout;
}

void controlResetATTITUDE (void){
	for(uint8_t i = 0; i < 3; i++) pidReset(&hpidAtt[i]);
}
