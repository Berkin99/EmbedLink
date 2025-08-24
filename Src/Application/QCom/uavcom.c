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

#include "xmath.h"
#include "xmath3d.h"
#include "navigation.h"
#include "uavcom.h"
#include "northcom.h"
#include "quadcopter.h"
#include "rtos.h"
#include "sysconfig.h"
#include "systime.h"
#include "uart.h"
#include "nrx.h"
#include "estimator.h"

#define STATE_ENTER       0x01
#define STATE_DURING      0x03
#define STATE_EXIT        0x02

#define STATE_CASE(state) (((uav_state == state) << 1) | (target_state == state))

taskAllocateStatic(UAVCOM, CONTROL_TASK_STACK, CONTROL_TASK_PRI);

/* Status */
static volatile uavcomState_e uav_state = UAVCOM_STATE_IDLE;
static volatile uavcomState_e next_state = UAVCOM_STATE_IDLE;

/* Commands */
static volatile uavcomState_e target_state = UAVCOM_STATE_IDLE;

static vec_t cpos; /* Command POS Buffer  */
static vec_t crot; /* Command ROT Buffer  */
static float ct;   /* Command Time Seconds */
static vec_t home;

void uavcomInit(void){
    taskCreateStatic(UAVCOM, uavcomTask, NULL);
}

void uavcomTask(void* argv){
    serialPrint("[>] UAVCOM Init : OK\n");
    
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
    float   cmdx[4] = {0.0f, 0.0f, 0.0f, 0.0f};

    cmd = data[0];
    for (int i = 0; i < 4; i++) memcpy(&cmdx[i], &data[(i * 4) + 1], 4);
    
    serialPrint("[%d], %.2f, %.2f, %.2f, %.2f\n", cmd, cmdx[0], cmdx[1], cmdx[2], cmdx[3]);
    
    switch (cmd){
        case UAVCOM_CMD_ARM:     uavcomArm(); break;
        case UAVCOM_CMD_DISARM:  uavcomDisarm(); break;
        case UAVCOM_CMD_TAKEOFF: uavcomTakeOff(cmdx[0], cmdx[1]); break;
        case UAVCOM_CMD_LAND:    uavcomLand(); break;
        case UAVCOM_CMD_MOVE:    uavcomMove(vnew(cmdx[0], cmdx[1],cmdx[2]),  cmdx[3]); break;
        case UAVCOM_CMD_YAW:     uavcomYaw(cmdx[0]); break;
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

void uavcomMove(vec_t pos, float t){
    target_state = UAVCOM_STATE_MOVING;
    cpos = pos;
    ct = clampf32(t, 0.1f, 100.0f) * 1000.0f;
}

void uavcomYaw(float yaw){
    crot.z = yaw;
}

void uavcomTakeOff(float z, float t){
    target_state = UAVCOM_STATE_TAKEOFF;
    cpos = xkinematicsState()->position.v;
    cpos.z = z;
    ct = clampf32(t, 1.0f, 100.0f) * 1000.0f;
}

void uavcomLand(void){
    target_state = UAVCOM_STATE_LAND;
}

void uavcomHome(void){
    target_state = UAVCOM_STATE_MOVING;
    cpos = home;
    ct = vdist(home, xkinematicsState()->position.v) * 0.5f * 1000.0f;
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
    xnavigationOrigin()->altitude = xnavigationState()->altitude;
    //estimatorOriginSet(); /* Updates the Z axis */
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
    
    static uint32_t mv_start;
    static vec_t a_pos, b_pos; /* Interval */
    static float mv_t;         /* Interval Time */
    static vec_t mvpos;        /* Gamma */

    if(!vequal(cpos, b_pos, 0.1f) && (statecase == STATE_DURING)) statecase = STATE_ENTER; /* Target updated */
    
    switch (statecase){
        case STATE_ENTER:
            if(quadGetMode() != QUAD_MODE_AUTO){target_state = uav_state; return;}
            next_state = UAVCOM_STATE_MOVING;
            /* Entered */
            mv_start = millis();
            a_pos = xkinematicsState()->position.v; /* Current */
            b_pos = cpos;  /* Target  */
            mv_t  = ct;
            break;
            serialPrint("[>] UAVCOM MOVING %.2f, %.2f, %.2f [%.2f]\n", b_pos.x, b_pos.y, b_pos.z, mv_t);
        
        case STATE_DURING:
            float elapsed = (float)(millis() - mv_start);
            float t = elapsed / mv_t; 
            mvpos = vlerp(a_pos, b_pos, t);
            quadcmd_AUTO(mvpos, crot.z);
            
            if (t > 1.0) target_state = UAVCOM_STATE_AUTO;

            break;
        case STATE_EXIT:

        break;
    }
}

void uavcomState_TAKEOFF(void){
    static uint32_t to_start;
    static float a_z, b_z; /* Interval */
    static float to_t;     /* Interval Time */
    static vec_t tpos;     /* Gamma */

    uint8_t statecase = STATE_CASE(UAVCOM_STATE_TAKEOFF);

    switch (statecase){
        case STATE_ENTER:{
            if((uav_state != UAVCOM_STATE_READY)
            || (quadSetMode(QUAD_MODE_AUTO) != OK)
            ){
                target_state = uav_state; return;
            }
            next_state = UAVCOM_STATE_TAKEOFF;
            /* Entered */
            to_start = millis();
            a_z  = xkinematicsState()->position.z;
            b_z  = cpos.z;
            to_t = ct;

            tpos = xkinematicsState()->position.v;
            home = cpos;
            quadcmd_AUTO(tpos, crot.z);
            serialPrint("[>] UAVCOM TAKEOFF %.2f [%.2f]\n", b_z, to_t);
        }break;
        case STATE_DURING:
            float elapsed = (float)(millis() - to_start);
            float t = elapsed / to_t;
            if (t > 1.0f) {
                target_state = UAVCOM_STATE_AUTO;
                break;
            }
            tpos.z = lerpf32(a_z, b_z, t);
            quadcmd_AUTO(tpos, crot.z);
        break;
    }
}

void uavcomState_LAND(void){
    uint8_t statecase = STATE_CASE(UAVCOM_STATE_LAND);

    static uint32_t ld_start;
    static float st_z;
    static float phase1_dur, phase2_dur;

    switch (statecase){
    case STATE_ENTER:
        if (quadGetMode() != QUAD_MODE_AUTO){ target_state = uav_state; return; }
        next_state = UAVCOM_STATE_LAND;
        ld_start = millis();

        st_z = xkinematicsState()->position.z;
        cpos  = xkinematicsState()->position.v;

        phase1_dur = (st_z * 1000.0f) / 1.5f;
        phase2_dur = 3000.0f;

        serialPrint("[>] UAVCOM LAND\n");
        break;
    case STATE_DURING: {
        float elapsed = (float)(millis() - ld_start);

        if (elapsed < phase1_dur){
            /* Phase 1: Ease-out : Fast to slow */
            float t = clampf32(elapsed / phase1_dur, 0.0f, 1.0f);
            float e = easeOutQuad(t);
            cpos.z = lerpf32(st_z, 0.5f, e);
        }
        else if (elapsed < phase1_dur + phase2_dur){
            /* Phase 2: */
            float t = clampf32((elapsed - phase1_dur) / phase2_dur, 0.0f, 1.0f);
            cpos.z = lerpf32(0.5f, -3.0f, t);
        }
        else {
            /* Land Complete */
            cpos.z = -3.0f;
            target_state = UAVCOM_STATE_READY;
            break;
        }

        quadcmd_AUTO(cpos, crot.z);
    } break;
    case STATE_EXIT:
        break;
    }
}

NRX_GROUP_START(uavcom)
NRX_ADD(NRX_UINT8, state, &uav_state)
NRX_ADD(NRX_UINT8, target, &target_state)
NRX_GROUP_STOP(uavcom)
