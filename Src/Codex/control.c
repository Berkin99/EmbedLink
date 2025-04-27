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

#include <systime.h>
#include "control.h"
#include "num.h"
#include "nrx.h"

static controller_t  controllerSystem;
static eventHandle_t modeChange;

controller_t* controller(void){
	return &controllerSystem;
}

typedef void (*control_f)(control_t ctrl);

void controllerSetRange    (control_t ctrl);
void controllerSetPower    (control_t ctrl);
void controllerSetAttitude (control_t ctrl);
void controllerSetVelocity (control_t ctrl);
void controllerSetPosition (control_t ctrl);

static control_f controlFunctions[CONTROL_COUNT] = {
	controllerSetRange,
	controllerSetPower,
	controllerSetAttitude,
	controllerSetVelocity,
	controllerSetPosition,
};

int8_t controllerModeSet (controller_e mode){
	if(mode >= CONTROL_MODE_COUNT) return E_OVERFLOW;
	if(controllerSystem.mode == mode) return E_OVERWRITE;
	controllerSystem.mode = mode;
	eventCall(&modeChange);
	return OK;
}

void   controllerModeCallBack (event_t callBack){
	eventAdd(&modeChange, callBack);
}

void controllerUpdate(control_t ctrl){
	if(ctrl.type > CONTROL_COUNT) return;
	controlFunctions[ctrl.type](ctrl);
	controllerSystem.lastUpdate = millis();
}

void   controllerTimeUpdate (void){
	controllerSystem.lastUpdate = millis();
}

void controllerSetRange    (control_t ctrl){controllerSystem.crange= ctrl.crange;}
void controllerSetPower    (control_t ctrl){controllerSystem.cpow  = ctrl.cpow;}
void controllerSetAttitude (control_t ctrl){controllerSystem.catt  = ctrl.catt;}
void controllerSetVelocity (control_t ctrl){controllerSystem.cvel  = ctrl.cvel;}
void controllerSetPosition (control_t ctrl){controllerSystem.cpos  = ctrl.cpos;}

int8_t controllerValidity(uint32_t timeout){
	if(millis() - controllerSystem.lastUpdate > timeout) return FALSE;
	return TRUE;
}

int8_t controllerIsValid(void){
	return controllerValidity(CONTROL_TIMEOUT_MS);
}

vec_t controlVec2Range(vec_t vector, float vscale){
	/* RANGE X :For moving y position, need to change negative pitch (range x) = [-]vector.y */
	/* RANGE Y :For moving x position, need to change positive roll  (range y) = [+]vector.x */
	vector = vdiv(vector, vscale);
	for (uint8_t i = 0; i < 3; ++i) {vector.axis[i] = constrainFloat(vector.axis[i], -1,  1);}
	return mkvec(-vector.y, vector.x, vector.z);
}
