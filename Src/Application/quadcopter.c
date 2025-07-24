// /*
//  *       ______          __             ____    _       __
//  *      / ____/___ ___  / /_  ___  ____/ / /   (_)___  / /__
//  *     / __/ / __ `__ \/ __ \/ _ \/ __  / /   / / __ \/ //_/
//  *    / /___/ / / / / / /_/ /  __/ /_/ / /___/ / / / / ,<
//  *   /_____/_/ /_/ /_/_.___/\___/\__,_/_____/_/_/ /_/_/|_|
//  *
//  *  EmbedLink Firmware
//  *  Copyright (c) 2024 Yeniay RD, All rights reserved.
//  *  _________________________________________________________
//  *
//  *  EmbedLink Firmware is free software: you can redistribute
//  *  it and/or  modify it under  the  terms of the  GNU Lesser
//  *  General Public License as  published by the Free Software
//  *  Foundation,  either version 3 of the License, or (at your
//  *  option) any later version.
//  *
//  *  EmbedLink  Firmware is  distributed  in the  hope that it
//  *  will be useful, but  WITHOUT  ANY  WARRANTY; without even
//  *  the implied warranty of MERCHANTABILITY or FITNESS FOR A
//  *  PARTICULAR PURPOSE.  See  the GNU  Lesser  General Public
//  *  License for more details.
//  *
//  *  You should have received a copy of the GNU Lesser General
//  *  Public License along with EmbedLink Firmware. If not, see
//  *  <http://www.gnu.org/licenses/>.
//  *
//  */

// #include <stdint.h>
// #include <math.h>

// #include "rtos.h"
// #include "system.h"
// #include "systime.h"
// #include "quadconfig.h"
// #include "quadcopter.h"
// #include "quadcal.h"
// #include "xmath.h"
// #include "quadcontrol.h"
// #include "kinematics.h"
// #include "led.h"
// #include "uart.h"
// #include "nrx.h"
// #include "northcom.h"
// #include "estimator.h"

// /** MOTOR MAPPING :
//  *  Front Right Motor : CCW : esc1 : FR
//  *  Rear Right Motor  : CW  : esc2 : RR
//  *  Rear Left Motor   : CCW : esc3 : RL
//  *  Front Left Motor  : CW  : esc4 : FL
//  */

// static quadcopter_t self;
// STATIC_MEM_TASK_ALLOC(QUAD,QUAD_TASK_STACK,QUAD_TASK_PRI)

// void quadInit(void){
//     self.mode = quadMode(QUAD_IDLE);

//     quadControlInit();
//     controllerModeCallBack(&quadControlModeCallBack);

//     for(uint8_t i = 0; i < 4; i++){self.motor[i] = ESC_NewHandle(ESC_PROTOCOL_STANDARD, i);}
//     quadStop(&self);

//     self.battery = BATT_NewHandle(&QBAT, 12.6f);
//     STATIC_MEM_TASK_CREATE(QUAD, QUAD_OPERATION, NULL);
// }

// void quadHealthCheck(quadcopter_t* pHandle){
//     if(!controllerIsValid() && pHandle->mode.modeid != QUAD_IDLE) quadSetMode(QUAD_IDLE);   /* Check last command time in ms */
// //    if(fabsf(kinematicsState()->rotation.x) > 90 || fabsf(kinematicsState()->rotation.y) > 90) quadSetMode(QUAD_IDLE);
// //    if(BATT_Voltage(&pHandle->battery) > 0.1f) quadSetMode(QUAD_IDLE);
// }

// void quadTask(void* argv){

//     systemWaitReady();
//     TickType_t xLastWakeTime = xTaskGetTickCount();
//     /* Should Run at exact 250 Hz */
//     while(1){
//         quadHealthCheck(&self);
//         quadmotor_t mout = self.mode.modeUpdate();
//         quadSetMotors(&self, mout);
//         vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(4));
//     }
// }

// void quadControlModeCallBack(void){
//     const static controller_e quadModeTransitionMap[CONTROL_MODE_COUNT] = {
//          [CONTROL_MODE_IDLE]   = QUAD_IDLE,
//          [CONTROL_MODE_MANUAL] = QUAD_MANUAL,
//          [CONTROL_MODE_AUTO]   = QUAD_AUTO,
// 		 [CONTROL_MODE_0]      = QUAD_HEIGHT,
//          [CONTROL_MODE_1]      = QUAD_AUTO,
//          [CONTROL_MODE_2]      = QUAD_TAKEOFF,
//          [CONTROL_MODE_3]      = QUAD_LAND,
//          [CONTROL_MODE_4]      = QUAD_IDLE,
//     };
//     quadSetMode(quadModeTransitionMap[controller()->mode]);
// }

// int8_t quadSetMode(quadmode_e mode){
//     if(mode >= QUAD_MODE_COUNT) return SYS_E_OVERFLOW;
//     /* Check mode transition map */
//     if(mode == QUAD_IDLE) ledSet(QLED, 0);
//     else ledSet(QLED, 1);

//     self.mode = quadMode(mode);
//     return SYS_OK;
// }

// int8_t quadSetMotors(quadcopter_t* pHandle, quadmotor_t cmd){
//     for (uint8_t i = 0; i < 4; i++) {
//         if(cmd.m[i] > 1.001f || cmd.m[i] < -0.001f) return SYS_E_OVERFLOW;
//         ESC_Write(&pHandle->motor[i], cmd.m[i]);
//     }
//     return SYS_OK;
// }

// void quadStop(quadcopter_t* pHandle){
//     const quadmotor_t zero = {{0.0f, 0.0f, 0.0f, 0.0f}};
//     quadSetMotors(pHandle, zero);
//     pHandle->mode = quadMode(QUAD_IDLE);
// }

// void quadCalibrate(void* argv){

//     systemWaitReady();

// //    TxMSG("[>] ESC CAL:");
// //    quadcalESC(&self);

// 	TxMSG("[>] NC CAL:");
//     if(quadcalIterate() > 0) quadcalCOM();
//     TxMSG("[>] MOTOR CAL:");
//     if(quadcalIterate() > 0) quadcalMotors(&self);
//     TxMSG("[>] SENSORS CAL:");
//     if(quadcalIterate() > 0) quadcalSensors(&self);
//     while(1){
//         TxMSG("[+] Calibration Complete");
//     	delay(5000);
//     }
// }