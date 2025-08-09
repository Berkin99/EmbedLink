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

#include "rtos.h"
#include "gpio.h"
#include "rccom.h"
#include "rc_interface.h"
#include "sysconfig.h"
#include "quadcopter.h"
#include "uart.h"

#define STATE_ENTER       0x01
#define STATE_DURING      0x03
#define STATE_EXIT        0x02
#define STATE_CASE(state) (((self_state == state) << 1) | (target_state == state))

taskAllocateStatic(RCCOM, CONTROL_TASK_STACK, CONTROL_TASK_PRI);

void _rccallback(uint8_t event);

static rccomState_e self_state   = RCCOM_STATE_IDLE;
static rccomState_e target_state = RCCOM_STATE_IDLE;
static rccomState_e next_state   = RCCOM_STATE_IDLE;

void rccomInit(void) {
    RC_SetCallback(_rccallback);
    serialPrint("[>] RCCOM Init : OK\n");
    taskCreateStatic(RCCOM, rccomTask, NULL);
}

void rccomTask(void* argv) {
    while (1) {
        rccomUpdate();
        delay(10);
    }
}

void rccomUpdate(void){
    if (rc.state == RC_ARMED){
        switch ((int)rc.chCONF.value){
            case 0: target_state = RCCOM_STATE_MANUAL; break;
            case 2: target_state = RCCOM_STATE_HEIGHT; break;
        }
    }

    RC_Validity();

    next_state = self_state;
    
    rccomState_IDLE();
    rccomState_MANUAL();
    rccomState_HEIGHT();
    
    self_state = next_state;
}


void rccomState_IDLE(void) {
    uint8_t statecase = STATE_CASE(RCCOM_STATE_IDLE);
        
    switch (statecase) {
        case STATE_ENTER:
            if (quadSetMode(QUAD_MODE_IDLE) == OK){
                next_state = RCCOM_STATE_IDLE;
                serialPrint("[>] RCCOM IDLE\n");
            }
            break;
        case STATE_DURING: break;
        case STATE_EXIT: break;
    }
}

void rccomState_MANUAL(void) {
    uint8_t statecase = STATE_CASE(RCCOM_STATE_MANUAL);
        
    switch (statecase) {
        case STATE_ENTER:
            if (quadSetMode(QUAD_MODE_MANUAL) != OK) return;
            next_state = RCCOM_STATE_MANUAL;
            serialPrint("[>] RCCOM MANUAL\n");
            break;    
        
        case STATE_DURING:
            vec_t range = vnew(rc.chX.value, rc.chY.value, rc.chZ.value);
            quadcmd_MANUAL(rc.chPOWER.value, range);
            break;

        case STATE_EXIT:break;
    }
}

void rccomState_HEIGHT(void) {
    uint8_t statecase = STATE_CASE(RCCOM_STATE_HEIGHT);

    static float t_height;
    switch (statecase) {
        case STATE_ENTER:
            if (quadSetMode(QUAD_MODE_HEIGHT) != OK) return; 
            next_state = RCCOM_STATE_HEIGHT;
            serialPrint("[>] RCCOM HEIGHT\n");
            t_height = xkinematicsState()->position.z;
            break;

        case STATE_DURING:
            vec_t range = vnew(rc.chX.value, rc.chY.value, rc.chZ.value);
            quadcmd_HEIGHT(rc.chPOWER.value, range, t_height);
            break;            
        case STATE_EXIT: break;
    }
}

void _rccallback(uint8_t event) {
    if (event == RC_EVENT_ARM) target_state = RCCOM_STATE_MANUAL;
    else target_state = RCCOM_STATE_IDLE;
}
