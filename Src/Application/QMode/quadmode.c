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

#include "systime.h"
#include "kinematics.h"
#include "quadmode.h"
#include "pid.h"
#include "xmath.h"
#include "xmath3d.h"
#include "navigation.h"
#include "control_attitude.h"
#include "control_height.h"
#include "control_navigation.h"
#include "uart.h"

#define QUAD_MODE_DEFINE(NAME) [QUAD_MODE_##NAME] = \
	{	.id = QUAD_MODE_##NAME,					\
		.modeUpdate = &quadTask_##NAME,				\
		.modePermission = &quadPermission_##NAME 	\
	}							\

static quadmode_t quadModes[] = {
	QUAD_MODE_DEFINE(IDLE),
	QUAD_MODE_DEFINE(READY),
	QUAD_MODE_DEFINE(MANUAL),
	QUAD_MODE_DEFINE(HEIGHT),
	QUAD_MODE_DEFINE(AUTO),
	QUAD_MODE_DEFINE(RAW)
};

void quadModeInit(void){
	controlInitATTITUDE();
	controlInitHEIGHT();
	controlInitNAV();
}

quadmode_t quadMode(quadmode_e id){
	return quadModes[id];
}

quadmotor_t quadTask_IDLE (quadcmd_t* pcmd){
	controlResetATTITUDE();
	controlResetHEIGHT();
	controlResetNAV();
	return (quadmotor_t){{0.0f, 0.0f, 0.0f, 0.0f}};
}

quadmotor_t quadTask_READY (quadcmd_t* pcmd){
	controlResetATTITUDE();
	controlResetHEIGHT();
	controlResetNAV();
	return (quadmotor_t){{0.1f, 0.1f, 0.1f, 0.1f}};
}

quadmotor_t quadTask_MANUAL(quadcmd_t* pcmd){
	controlResetHEIGHT();
	controlResetNAV();
	return controlTaskATTITUDE(pcmd->cpow, pcmd->crange);
}

quadmotor_t quadTask_HEIGHT(quadcmd_t* pcmd){
	controlResetNAV();
	return controlTaskATTITUDE(controlTaskHEIGHT(pcmd->cpos.z), pcmd->crange);
}

quadmotor_t quadTask_AUTO(quadcmd_t* pcmd){
//	vec_t vnet = vrot2(controlTaskNAV(pcmd->cpos), -navigationState()->compass * DEG2RAD);
    vec_t vnet = vrot2(controlTaskNAV(pcmd->cpos), -xkinematicsState()->rotation.z * DEG2RAD); /* NOCOMPASS POSITIONING */

	vnet = vdiv(vnet, 10.0f);
	for (uint8_t i = 0; i < 3; ++i) {vnet.axis[i] = clampf32(vnet.axis[i], -1,  1);}
	vec_t range = vnew(-vnet.y, vnet.x, vnet.z);

	range.z = clampf32(pcmd->crot.z / 180.0f, -1.0, 1.0); /* Target Z angle [-1, 1] */
	return controlTaskATTITUDE(controlTaskHEIGHT(pcmd->cpos.z), range);
}

quadmotor_t quadTask_RAW (quadcmd_t* pcmd){
	quadmotor_t motors = {{pcmd->craw[0], pcmd->craw[1], pcmd->craw[2], pcmd->craw[3]}};
	for (uint8_t i = 0; i < 4; i++)  motors.m[i] = clampf32(motors.m[i], 0.0f, 1.0f);
	return motors;
}

int8_t quadPermission_IDLE   (quadmode_e lmode){return 1;}

int8_t quadPermission_READY  (quadmode_e lmode){return 1;}

int8_t quadPermission_MANUAL (quadmode_e lmode){
	int8_t permission = 
	xkinematicsIsValid(KINV_IATTITUDE, 1000) &&
	xkinematicsIsValid(KINV_ROTATION, 1000);
	return permission;
}

int8_t quadPermission_HEIGHT (quadmode_e lmode){
	int8_t permission = 
	quadPermission_MANUAL(QUAD_MODE_MANUAL) &&
	xkinematicsIsValid(KINV_POSITION, 1000) &&
	(xkinematicsState()->position.stdDev.z >= 0.0f);
	return permission;
}

int8_t quadPermission_AUTO   (quadmode_e lmode){
	int8_t permission = 
	quadPermission_HEIGHT(QUAD_MODE_HEIGHT) &&
	xkinematicsState()->position.stdDev.x >= 0.0f &&
	xkinematicsState()->position.stdDev.y >= 0.0f &&
	xnavigationState()->location.latitude != 0.0f;    /* GPS CONTROL */
	return permission;
}

int8_t quadPermission_RAW    (quadmode_e lmode){return 1;}
