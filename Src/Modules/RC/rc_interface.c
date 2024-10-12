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
#include "control.h"
#include "rc_interface.h"
#include "systime.h"
#include "num.h"
#include "kinematics.h"

RC_Handle_t RC_NewHandle(){

    const RC_Channel_t midCfg  = {0, {RC_CH_DEADBAND, RC_CH_CENTER, -127,  127}};
    const RC_Channel_t defCfg  = {0, {0, 0, 0, 255}};
    const RC_Channel_t zeroCfg = {0, {0, 0, 0, 0}};

    RC_Handle_t tmp;
    tmp.state = RC_DISARMED;

    tmp.lastUpdate = millis();

    tmp.ch[RC_CH_X] = midCfg;
    tmp.ch[RC_CH_Y] = midCfg;
    tmp.ch[RC_CH_Z] = midCfg;
    tmp.ch[RC_CH_POWER] = defCfg;
    tmp.ch[RC_CH_CONF]  = zeroCfg;

    return tmp;
}

void  RC_Calibrate (RC_Handle_t* pRC){
	/* Memory */
}

void  RC_Update (RC_Handle_t* pRC, uint8_t raw[5]){

    for (uint8_t i = 0; i < RC_CH_LENGTH; ++i) RC_ChannelAlign(&pRC->ch[i], raw[i]);
    pRC->lastUpdate = millis();

    RC_Control(pRC);

    if(pRC->chPOWER.value > 0.05) return; /* Switch Area */

    if(pRC->state == RC_ARMED){
        if(pRC->chPOWER.value == 0 && pRC->chZ.value < -0.9f){
            pRC->state = RC_DISARMED;
        }
    }
    else{
        if(pRC->chPOWER.value == 0 && pRC->chZ.value > 0.9f) pRC->state = RC_SWITCH;
        if(pRC->state == RC_SWITCH && pRC->chZ.value < 0.1) {
            pRC->state = RC_ARMED;
        }
    }
}

void RC_Control (RC_Handle_t* pRC){

	control_t   range;
	control_t   power;
	power.type   = CONTROL_POWER;
	range.type   = CONTROL_RANGE;
	power.cpow   = pRC->chPOWER.value;
	range.crange = mkvec(pRC->chX.value,  pRC->chY.value,  pRC->chZ.value);

	controllerUpdate(range);
	controllerUpdate(power);

	if(pRC->state != RC_ARMED){ controllerModeSet(CONTROL_MODE_IDLE); return; }

    if(pRC->chCONF.value == 1){     /* HEIGHT */
    	if(controllerModeSet(CONTROL_MODE_0) == OK){
			control_t hold;
			hold.type = CONTROL_POSITION;
			hold.cpos = mkvec(0, 0, 2.0f);
			controllerUpdate(hold);
    	}
    }
    else if(pRC->chCONF.value == 2){ /* AUTO */
    	if(controllerModeSet(CONTROL_MODE_1) == OK){
			control_t hold;
			hold.type = CONTROL_POSITION;
			hold.cpos = mkvec(kinematicsState()->position.x, kinematicsState()->position.y, 2.0f);
			controllerUpdate(hold);
    	}
    }
    else if(pRC->chCONF.value == 3){ /* LAND */
    	controllerModeSet(CONTROL_MODE_2);
    }
    else controllerModeSet(CONTROL_MODE_MANUAL);
}

void RC_Validity(RC_Handle_t* pRC){
	if(millis() - pRC->lastUpdate > RC_CONNECTION_LOST_TIME_MS){
        pRC->state = RC_DISARMED;
	}
}

/* [-1,1] */
void RC_ChannelAlign(RC_Channel_t* ch, uint8_t raw){

    float val = (float) raw;

	if(ch->settings.max == 0){ /* Digital */
		ch->value = val;
		return;
	}

    val -= ch->settings.center;
    val = deadbandFloat(val, ch->settings.deadband);

    if     (ch < 0) val /= fabsf(ch->settings.min);
    else if(ch > 0) val /= ch->settings.max;

    ch->value = constrainFloat(val, -1, 1);
}
