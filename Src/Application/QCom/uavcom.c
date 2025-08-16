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

#include <string.h>

#include "navigation.h"
#include "uavcom.h"
#include "northcom.h"
#include "quadcopter.h"
#include "rtos.h"
#include "sysconfig.h"
#include "systime.h"
#include "uart.h"
#include "nrx.h"

#define STATE_ENTER       0x01
#define STATE_DURING      0x03
#define STATE_EXIT        0x02

#define STATE_CASE(state) (((uav_state == state) << 1) | (target_state == state))

taskAllocateStatic(UAVCOM, CONTROL_TASK_STACK, CONTROL_TASK_PRI);

/* Status */
uavcomState_e uav_state = UAVCOM_STATE_IDLE;
static uavcomState_e next_state = UAVCOM_STATE_IDLE;

/* Commands */
static uavcomState_e target_state = UAVCOM_STATE_IDLE;
static vec_t cpos;
static vec_t crot;
static vec_t home;

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

    next_state = uav_state;
    
    uavcomState_IDLE();
    uavcomState_READY();
    uavcomState_AUTO();
    uavcomState_MOVING();
    uavcomState_TAKEOFF();
    uavcomState_LAND();

    uav_state = next_state;
}

void uavcomParse(uint8_t* data){
    uint8_t cmd;
    vec_t cmdv;

    cmd = data[0];
    for (int i = 0; i < 3; i++) memcpy(&cmdv.axis[i], &data[(i * 4) + 1], 4);

    switch (cmd){
        case UAVCOM_CMD_ARM:     uavcomArm(); break;
        case UAVCOM_CMD_DISARM:  uavcomDisarm(); break;
        case UAVCOM_CMD_TAKEOFF: uavcomTakeOff(cmdv.axis[0]); break;
        case UAVCOM_CMD_LAND:    uavcomLand(); break;
        case UAVCOM_CMD_MOVE:    uavcomMove(cmdv); break;
        case UAVCOM_CMD_YAW:     uavcomYaw(cmdv.axis[0]); break;
        case UAVCOM_CMD_HOME:    uavcomHome(); break;
        case UAVCOM_CMD_KILL:    uavcomKill(); break;
        case UAVCOM_CMD_ORIGIN:  uavcomOrigin(&data[1]); break;
    }
}

void uavcomArm(void){ 
    target_state = UAVCOM_STATE_READY; 
}

void uavcomDisarm(void){
    target_state = UAVCOM_STATE_IDLE; 
}

void uavcomMove(vec_t pos){
    target_state = UAVCOM_STATE_MOVING;
    cpos = pos;
}

void uavcomYaw(float yaw){
    crot.z = yaw;
}

void uavcomTakeOff(float z){
    target_state = UAVCOM_STATE_TAKEOFF;
    cpos.z = z;
}

void uavcomLand(void){
    target_state = UAVCOM_STATE_LAND;
}

void uavcomHome(void){
    target_state = UAVCOM_STATE_MOVING;
    cpos = home;
}

void uavcomKill(void){ 
    quadSetMode(QUAD_MODE_IDLE);
    target_state = UAVCOM_STATE_IDLE;    
    uav_state = UAVCOM_STATE_IDLE; 
}

void uavcomOrigin(uint8_t* data){
    f64 loc[2];
    for (int i = 0; i < 2; i++) memcpy(&loc[i], &data[i * sizeof(f64)], sizeof(f64));
    xnavigationOrigin()->location.latitude = loc[0];
    xnavigationOrigin()->location.longitude = loc[1];
}

void uavcomState_IDLE(void){
    uint8_t statecase = STATE_CASE(UAVCOM_STATE_IDLE);
    switch (statecase){
        case STATE_ENTER:
            quadSetMode(QUAD_MODE_IDLE);
            next_state = UAVCOM_STATE_IDLE;
            serialPrint("[>] UAVCOM IDLE\n");
            /* Entered */
        break;
    }
}

void uavcomState_READY(void){
    uint8_t statecase = STATE_CASE(UAVCOM_STATE_READY);
    switch (statecase){
        case STATE_ENTER:
            if(quadSetMode(QUAD_MODE_READY) != OK) return;
            next_state = UAVCOM_STATE_READY;
            serialPrint("[>] UAVCOM READY\n");
            /* Entered */
        break;
    }
}

void uavcomState_AUTO(void){
    uint8_t statecase = STATE_CASE(UAVCOM_STATE_AUTO);
    switch (statecase){
        case STATE_ENTER:
            next_state = UAVCOM_STATE_AUTO;
            serialPrint("[>] UAVCOM AUTO\n");
            /* Entered */
        break;
        case STATE_DURING:
            quadcmd_AUTO(cpos, crot.z);
        break;
        case STATE_EXIT:

        break;
    }
}

void uavcomState_MOVING(void){
    uint8_t statecase = STATE_CASE(UAVCOM_STATE_MOVING);
    static uint8_t  arrived;
    static uint32_t arrive_t;

    switch (statecase){
        case STATE_ENTER:
            if(quadGetMode() != QUAD_MODE_AUTO){target_state = uav_state; return;}
            next_state = UAVCOM_STATE_MOVING;
            /* Entered */
            arrived  = FALSE;
            arrive_t = millis();
            serialPrint("[>] UAVCOM MOVING %.2f, %.2f, %.2f\n", cpos.x, cpos.y, cpos.z);
        break;
        case STATE_DURING:
            quadcmd_AUTO(cpos, crot.z);
            arrived = vdist(cpos, xkinematicsState()->position.v) < UAV_ARRIVAL_DISTANCE;
            if(arrived){
                if(millis() - arrive_t > UAV_ARRIVAL_COUNTER_MS) target_state = UAVCOM_STATE_AUTO;
            }
            else arrive_t = millis();
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
            if(uav_state != UAVCOM_STATE_READY) return;
            if(quadSetMode(QUAD_MODE_AUTO) != OK) return;
            next_state = UAVCOM_STATE_TAKEOFF;
            /* Entered */
            to_start = millis();
            to_z = cpos.z;
            st_z = xkinematicsState()->position.z;
            cpos = xkinematicsState()->position.v;
            home = xkinematicsState()->position.v;
            st_yaw = xkinematicsState()->rotation.z;
            serialPrint("[>] UAVCOM TAKEOFF %.2f\n", to_z);
        break;
        case STATE_DURING:

            float ivar = (float)(millis() - to_start) / 6000.0f;
            if (ivar > 1.0f) {
                target_state = UAVCOM_STATE_AUTO;
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
    
    static uint32_t ld_start;
    static float ld_z, st_z, st_yaw;

    switch (statecase){
        case STATE_ENTER:
            if(quadGetMode() != QUAD_MODE_AUTO){target_state = uav_state; return;}
            next_state = UAVCOM_STATE_LAND;
            ld_start = millis();
            ld_z = -3.0f;
            st_z = xkinematicsState()->position.z;
            cpos = xkinematicsState()->position.v;
            st_yaw = xkinematicsState()->rotation.z;
            serialPrint("[>] UAVCOM LAND\n");

        break;
        case STATE_DURING:
            float ivar = (float)(millis() - ld_start) / 8000.0f;
            if(ivar > 1.0f){
                target_state = UAVCOM_STATE_READY;
                break;
            }
            cpos.z = ivar * ld_z  + (1.0f - ivar) * st_z;
            quadcmd_AUTO(cpos, st_yaw);
        break;
        case STATE_EXIT:
        
        break;
    }
}

// NRX_GROUP_START(uavcom)
// NRX_ADD(NRX_UINT8, "state", &uav_state)
// NRX_ADD(NRX_UINT8, "target", &target_state)
// NRX_GROUP_STOP(uavcom)
