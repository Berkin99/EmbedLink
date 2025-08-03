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
#include "quadcontrol.h"
#include "kinematics.h"
#include "led.h"
#include "uart.h"
#include "estimator.h"
#include "nrx.h"
#include "northcom.h"

/** MOTOR MAPPING :
 *  Front Right Motor : CCW : esc1 : FR
 *  Rear Right Motor  : CW  : esc2 : RR
 *  Rear Left Motor   : CCW : esc3 : RL
 *  Front Left Motor  : CW  : esc4 : FL
 */

static quadcopter_t self;

taskAllocateStatic(QUAD, QUAD_TASK_STACK, QUAD_TASK_PRI);

void quadInit(void){
    quadSetMode(QUAD_IDLE);
    quadControlInit();
    
    self.motor[0] = ESC_NewHandle(&pwm1, ESC_PROTOCOL_STANDARD);
    self.motor[1] = ESC_NewHandle(&pwm2, ESC_PROTOCOL_STANDARD);
    self.motor[2] = ESC_NewHandle(&pwm3, ESC_PROTOCOL_STANDARD);
    self.motor[3] = ESC_NewHandle(&pwm4, ESC_PROTOCOL_STANDARD);
    
    for (int i = 0; i < 4; i++) ESC_Start(&self.motor[i]);
    
    quadStop(&self);

    taskCreateStatic(QUAD, QUAD_OPERATION, NULL);
}

void quadHealthCheck(quadcopter_t* pHandle){
    //if(!controllerIsValid() && pHandle->mode.modeid != QUAD_IDLE) quadSetMode(QUAD_IDLE);   /* Check last command time in ms */
    //if(fabsf(kinematicsState()->rotation.x) > 90 || fabsf(kinematicsState()->rotation.y) > 90) quadSetMode(QUAD_IDLE);
    //if(BATT_Voltage(&pHandle->battery) > 0.1f) quadSetMode(QUAD_IDLE);
}

void quadTask(void* argv){
    systemWaitReady();
    uint32_t waketimer = taskGetTickCount();
    /* Should Run at exact 250 Hz */
    while(1){
        quadHealthCheck(&self);
        quadmotor_t mout = self.mode.modeUpdate();
        quadSetMotors(&self, mout);
        taskDelayUntil(&waketimer, 4);
    }
}

int8_t quadSetMode(quadmode_e mode){
    if(mode >= QUAD_MODE_COUNT) return E_OVERFLOW;
    /* Check mode transition map */
    if(mode == QUAD_IDLE) ledSet(QLED, 0);
    else ledSet(QLED, 1);

    self.mode = quadMode(mode);
    return OK;
}

int8_t quadSetMotors(quadcopter_t* pHandle, quadmotor_t cmd){
//    serialPrint("[>] ");
    for (uint8_t i = 0; i < 4; i++) {
        if(cmd.m[i] > 1.001f || cmd.m[i] < -0.001f) return E_OVERFLOW;
        ESC_Write(&pHandle->motor[i], cmd.m[i]);
        //serialPrint(" %.2f", cmd.m[i]);
    }
//    serialPrint("\n");
    return OK;
}

void quadStop(quadcopter_t* pHandle){
    const quadmotor_t zero = {{0.0f, 0.0f, 0.0f, 0.0f}};
    quadSetMotors(pHandle, zero);
    pHandle->mode = quadMode(QUAD_IDLE);
}

void quadCalibrate(void* argv){

    systemWaitReady();

    //    TxMSG("[>] ESC CAL:");
    //    quadcalESC(&self);

    //	TxMSG("[>] NC CAL:");
    //    if(quadcalIterate() > 0) quadcalCOM();
    //    TxMSG("[>] MOTOR CAL:");
    //    if(quadcalIterate() > 0) quadcalMotors(&self);
    //    TxMSG("[>] SENSORS CAL:");
    //    if(quadcalIterate() > 0) quadcalSensors(&self);
    //    while(1){
    //       TxMSG("[+] Calibration Complete");
    //    	delay(5000);
    //    }

}
