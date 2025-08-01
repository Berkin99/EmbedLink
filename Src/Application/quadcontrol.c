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
#include "quadcontrol.h"
#include "pid.h"
#include "xmath.h"
#include "xmath3d.h"
#include "navigation.h"

#include "control_attitude.h"
#include "control_height.h"
#include "control_navigation.h"
#include "control.h"

/* TODO: Configuring the demands lists can prevent undesirable behaviors */

//static kinematics_e idleDemands[] = {};
//const state_e ManualDemands[] = {STATE_INTERNAL_ATTITUDE, STATE_ROTATION};
//const state_e HeightDemands[] = {STATE_INTERNAL_ATTITUDE, STATE_ROTATION};
//const state_e AutoDemands[]   = {STATE_INTERNAL_ATTITUDE, STATE_ROTATION, STATE_POSITION};

static quadmode_t quadModes[] = {
	[QUAD_IDLE] =
	{	.modeid = QUAD_IDLE,
		.modeUpdate = &quadIdle
	},
	[QUAD_READY] =
	{	.modeid = QUAD_READY,
		.modeUpdate = &quadReady
	},
	[QUAD_MANUAL] =
	{	.modeid = QUAD_MANUAL,
		.modeUpdate = &quadManual
	},
	[QUAD_HEIGHT] =
	{	.modeid = QUAD_HEIGHT,
		.modeUpdate = &quadManualHeight
	},
	[QUAD_AUTO] =
	{	.modeid = QUAD_AUTO,
		.modeUpdate = &quadAutoNav
	},
	[QUAD_TAKEOFF] =
	{   .modeid = QUAD_TAKEOFF,
		.modeUpdate = &quadTakeOff
	},
	[QUAD_LAND] =
	{   .modeid = QUAD_LAND,
		.modeUpdate = &quadLand
	},
};

static controller_t* pctrl;
static quadmotor_t   mzero = {{0, 0, 0, 0}};

void quadControlInit (void){
	pctrl = controller();
	controlInitATTITUDE();
	controlInitHEIGHT();
	controlInitNAV();
}

quadmode_t quadMode(quadmode_e modeid){
	if(modeid > QUAD_MODE_COUNT) return quadModes[QUAD_IDLE];
	return quadModes[modeid];
}

quadmotor_t quadIdle (void){
	controlResetATTITUDE();
	controlResetHEIGHT();
	controlResetNAV();
	return mzero;
}

quadmotor_t quadManual(void){
	controlResetHEIGHT();
	controlResetNAV();
	return controlTaskATTITUDE(pctrl->cpow, pctrl->crange);
}

quadmotor_t quadManualHeight(void){
	controlResetNAV();
	return controlTaskATTITUDE(controlTaskHEIGHT(pctrl->cpos.z), pctrl->crange);
}

quadmotor_t quadAutoNav(void){
//	vec_t vnet = vrot2(controlTaskNAV(pctrl->cpos), -navigationState()->compass * DEG2RAD);
    vec_t vnet = vrot2(controlTaskNAV(pctrl->cpos), -xkinematicsState()->rotation.z * DEG2RAD); /* NOCOMPASS POSITIONING */

	vnet = vdiv(vnet, 10.0f);
	for (uint8_t i = 0; i < 3; ++i) {vnet.axis[i] = clampf32(vnet.axis[i], -1,  1);}
	controlRange_t rangeNav = vnew(-vnet.y, vnet.x, vnet.z);

	rangeNav.z = 0.0f; /* Target Z angle [-1, 1] */
	return controlTaskATTITUDE(controlTaskHEIGHT(pctrl->cpos.z), rangeNav);
}

quadmotor_t quadTakeOff(void){
	static uint32_t lastTakeoff;  /* Milliseconds */
	static vec_t 	posTakeoff;

	if(millis() - lastTakeoff > 100){
		posTakeoff = xkinematicsState()->position.v;
		posTakeoff.z = 2.0f;
	}

	if(xkinematicsState()->position.z > 1.5f){
		control_t ct; ct.type = CONTROL_POSITION; ct.cpos = posTakeoff;
		controllerUpdate(ct);
		quadSetMode(QUAD_AUTO);
	}

	lastTakeoff = millis();
	controlResetNAV();
	return controlTaskATTITUDE(controlTaskHEIGHT(2.0f), vzero());
}

quadmotor_t quadLand (void){
	static float landingTimer; /* Seconds */
	static uint32_t  lastLand; /* Milliseconds */
	static vec_t      posLand;

	if(millis() - lastLand > 100){landingTimer = 0; posLand = xkinematicsState()->position.v;} /* New Land Command Landing position set */
	if(xkinematicsState()->position.z > 1.5f) landingTimer = 0; else{landingTimer += 0.004;}        /* Count the timer when under 1.2 meters */
	if(landingTimer > 4.0f){landingTimer = 0; quadSetMode(QUAD_IDLE);}                             /* Landing Timer Exceeds 4 sec */

	lastLand = millis();
	/* @Return */
	if(xkinematicsState()->position.z > 1.5f){
		pctrl->cpos = posLand;
		pctrl->cpos.z = 1.3f;
		return quadAutoNav();
	}
	else{
		posLand.z = xkinematicsState()->position.z - 0.25f - (landingTimer / 3.0f);
		controlResetNAV();
		return controlTaskATTITUDE(controlTaskHEIGHT(posLand.z), vzero());
	}
}
