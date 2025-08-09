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

#include <stdint.h>
#include <math.h>

#include "rtos.h"
#include "task.h"
#include "system.h"
#include "systime.h"
#include "quadconfig.h"
#include "quadcopter.h"
#include "quadcal.h"
#include "xmath.h"
#include "kinematics.h"
#include "led.h"
#include "uart.h"
#include "estimator.h"
#include "nrx.h"
#include "northcom.h"
#include "uart.h"

/** MOTOR MAPPING :
 *  Front Right Motor : CCW : esc1 : FR
 *  Rear Right Motor  : CW  : esc2 : RR
 *  Rear Left Motor   : CCW : esc3 : RL
 *  Front Left Motor  : CW  : esc4 : FL
 */

static quadcopter_t self;

taskAllocateStatic(QUAD, QUAD_TASK_STACK, QUAD_TASK_PRI);

int8_t quadInit(void){
    
    self.motor[0] = ESC_NewHandle(&pwm1, ESC_PROTOCOL_STANDARD);
    self.motor[1] = ESC_NewHandle(&pwm2, ESC_PROTOCOL_STANDARD);
    self.motor[2] = ESC_NewHandle(&pwm3, ESC_PROTOCOL_STANDARD);
    self.motor[3] = ESC_NewHandle(&pwm4, ESC_PROTOCOL_STANDARD);
    
    for (int i = 0; i < 4; i++) ESC_Start(&self.motor[i]);
    quadModeInit();
    quadStop();
    
    taskCreateStatic(QUAD, QUAD_OPERATION, NULL);

    return OK;
}

void quadHealthCheck(void){
    //if(!controllerIsValid() && pHandle->mode.id != QUAD_IDLE) quadSetMode(QUAD_IDLE);   /* Check last command time in ms */
    //if(fabsf(kinematicsState()->rotation.x) > 90 || fabsf(kinematicsState()->rotation.y) > 90) quadSetMode(QUAD_IDLE);
    //if(BATT_Voltage(&pHandle->battery) > 0.1f) quadSetMode(QUAD_IDLE);
}

void quadTask(void* argv){
    /* MAIN TASK */
    /** @warning:  Run at exact 250 Hz */
    
    systemWaitReady();
    uint32_t waketimer = taskGetTickCount();
    while(1){
        if(!self.mode.modePermission(self.mode.id)) quadStop();   /* Check for mode demands */
        quadHealthCheck();                                  /* Check for quadcopter elementary health */
        quadmotor_t mout = self.mode.modeUpdate(&self.cmd); /* Calculate the motor powers */
        quadSetMotors(mout);                                /* Apply the power */
        taskDelayUntil(&waketimer, 4); 
    }
}

int8_t quadSetMode(quadmode_e mode){
    if(!quadMode(mode).modePermission(self.mode.id)) return E_ERROR;
    self.mode = quadMode(mode);
    #ifdef QUAD_DEBUG
    serialPrint("[>] QUADMODE : %d\n", self.mode.id);
    #endif
    return OK;
}

void quadSetMotors(quadmotor_t cmd){
    #ifdef QUAD_DEBUG
    serialPrint("[>] ");
    for (uint8_t i = 0; i < 4; i++) {
        if(cmd.m[i] > 1.001f || cmd.m[i] < -0.001f) return;
        ESC_Write(&self.motor[i], cmd.m[i]);
        serialPrint(" %.2f", cmd.m[i]);
    }
    serialPrint("\n");
    #else
    for (uint8_t i = 0; i < 4; i++) {
        if(cmd.m[i] > 1.001f || cmd.m[i] < -0.001f) return;
        ESC_Write(&self.motor[i], cmd.m[i]);
    }
    #endif
}

void quadStop(void){
    const quadmotor_t zero = {{0.0f, 0.0f, 0.0f, 0.0f}};
    quadSetMotors(zero);
    quadSetMode(QUAD_MODE_IDLE);
}

void quadCalibrate(void* argv){

    systemWaitReady();

    TxMSG("[>] ESC CAL:");
    quadcalESC(&self);

    TxMSG("[>] NC CAL:");
    if(quadcalIterate() > 0) quadcalCOM();
    TxMSG("[>] MOTOR CAL:");
    if(quadcalIterate() > 0) quadcalMotors(&self);
    TxMSG("[>] SENSORS CAL:");
    if(quadcalIterate() > 0) quadcalSensors(&self);
    while(1){
        TxMSG("[+] Calibration Complete");
        delay(5000);
    }
}

void quadcmd_MANUAL(float cpow, vec_t crange){
    self.cmd.cpow = cpow;
    self.cmd.crange = crange;
}

void quadcmd_HEIGHT(float cpow, vec_t crange, float z){
    quadcmd_MANUAL(cpow, crange);
    self.cmd.cpos.z = z;
}

void quadcmd_AUTO(vec_t cpos, float yaw){
    self.cmd.cpos = cpos;
    self.cmd.crot.z = yaw;
}

void quadcmd_RAW(float craw[4]){
    memcpy(&self.cmd.craw, craw, 4 * (sizeof(float)));
}