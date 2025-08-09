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


#include "uavcom.h"
#include "northcom.h"
#include "quadcopter.h"
#include "rtos.h"
#include "sysconfig.h"

#define STATE_ENTER       0x01
#define STATE_DURING      0x03
#define STATE_EXIT        0x02
#define STATE_CASE(state) (((self_state == state) << 1) | (target_state == state))

taskAllocateStatic(UAVCOM, CONTROL_TASK_STACK, CONTROL_TASK_PRI);

static uavcomState_e self_state   = UAVCOM_STATE_IDLE;
static uavcomState_e next_state   = UAVCOM_STATE_IDLE;

static uavcomState_e target_state = UAVCOM_STATE_IDLE;
static vec_t cpos;

void uavcomInit(void){
    serialPrint("[>] UAVCOM Init : OK\n");
    taskCreateStatic(UAVCOM, uavcomTask, NULL);
}

void uavcomTask(void* argv){
    while (1){
        uavcomUpdate();
        delay(10);
    }
}

void uavcomUpdate(void){
    next_state = self_state;
    
    uavcomState_IDLE();
    uavcomState_READY();
    uavcomState_AUTO();
    uavcomState_TAKEOFF();
    uavcomState_LAND();

    self_state = next_state;
}

void uavcomArm(void){ target_state = UAVCOM_STATE_READY; }

void uavcomDisarm(void){ target_state = UAVCOM_STATE_IDLE; }

void uavcomTakeOff(float z){
    cpos.z = z;
    target_state = UAVCOM_STATE_TAKEOFF;
}

void uavcomLand(void);

void uavcomPose(vec_t pos, float yaw){
    quadcmd_AUTO(pos, yaw);
}

void uavcomKill(void){ self_state = UAVCOM_STATE_IDLE; }

void uavcomState_IDLE(void){
    uint8_t statecase = STATE_CASE(UAVCOM_STATE_IDLE);
    switch (statecase){
        case STATE_ENTER:
            /* code */
        break;
        case STATE_DURING:

        break;
        case STATE_EXIT:

        break;
    }
}

void uavcomState_READY(void){
    uint8_t statecase = STATE_CASE(UAVCOM_STATE_READY);
    switch (statecase){
        case STATE_ENTER:
            /* code */
        break;
        case STATE_DURING:

        break;
        case STATE_EXIT:

        break;
    }
}

void uavcomState_AUTO(void){
    uint8_t statecase = STATE_CASE(UAVCOM_STATE_AUTO);
    switch (statecase){
        case STATE_ENTER:
            /* code */
        break;
        case STATE_DURING:

        break;
        case STATE_EXIT:

        break;
    }
}

void uavcomState_TAKEOFF(void){
    uint8_t statecase = STATE_CASE(UAVCOM_STATE_TAKEOFF);
    static uint32_t to_start;
    static float to_z, st_z, st_yaw;
    switch (statecase){
        case STATE_ENTER:
            if(self_state != UAVCOM_STATE_READY) return;
            if(quadSetMode(QUAD_MODE_AUTO) == OK) return;
            next_state = UAVCOM_STATE_TAKEOFF;
            to_start = millis();
            to_z = cpos.z;
            st_z = xkinematicsState()->position.z;
            cpos = xkinematicsState()->position.v;
            st_yaw = xkinematicsState()->rotation.z;
        break;
        case STATE_DURING:
            float ivar = (float)(millis() - to_start) / 5000.0f;
            if (ivar > 1.0f) {
                next_state = UAVCOM_STATE_AUTO;
                break;
            }
            cpos.z = ivar * to_z  + (1.0f - ivar) * st_z;

            quadcmd_AUTO(cpos, st_yaw);
        break;
        case STATE_EXIT:

        break;
    }
}

void uavcomState_LAND(void){
    uint8_t statecase = STATE_CASE(UAVCOM_STATE_LAND);
    switch (statecase){
        case STATE_ENTER:
            /* code */
        break;
        case STATE_DURING:

        break;
        case STATE_EXIT:

        break;
    }
}
