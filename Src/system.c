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

#include "system.h"
#include "systime.h"
#include "sysconfig.h"

#include "adc.h"
#include "gpio.h"
#include "i2c.h"
#include "irq.h"
#include "pwm.h"
#include "rtos.h"
#include "spi.h"
#include "uart.h"

#include "sensor.h"
#include "estimator.h"
#include "telemetry.h"
#include "navigator.h"
#include "memory.h"

#include "ledseq.h"
#include "led.h"
#include "esc.h"
#include "watchtime.h"
#include "quadcopter.h"
#include "kinematics.h"
#include "navigation.h"

#include "northcom.h"
#include "rccom.h"
#include "uavcom.h"
#include "uavexe.h"

static uint8_t sysInit = 0;

taskAllocateStatic(SYSTEM_TASK, SYSTEM_TASK_STACK, SYSTEM_TASK_PRI);
void systemTask(void* argv);

void systemLaunch(void){
    if(sysInit) return;
    sysInit = 1;

    taskCreateStatic(SYSTEM_TASK, systemTask, NULL);
    taskStartScheduler();
    /* Should not reach here */
    systemErrorCall();
    while(1);
}

void systemTask(void* argv){

    i2cInit();
    pwmInit();
    spiInit();
    uartInit();
    irqInit();

    ledseqInit();
    ledseqRun(LED1, 1, SEQ_PROCESS_L);
    ledseqRun(LED2, 1, SEQ_HEARTBEAT);

    delay(600);

    serialPrint("[>] System Start\n");

    quadInit();

    memoryInit();
    memoryTest();
    memoryClear();
    //memoryDownload();

    sensorInit();
    sensorTest();
    telemetryInit();
    telemetryTest();
    navigatorInit();
    navigatorTest();

    // ESC_Handle_t m[4];
    // m[0] = ESC_NewHandle(&pwm1, ESC_PROTOCOL_STANDARD);
    // m[1] = ESC_NewHandle(&pwm2, ESC_PROTOCOL_STANDARD);
    // m[2] = ESC_NewHandle(&pwm3, ESC_PROTOCOL_STANDARD);
    // m[3] = ESC_NewHandle(&pwm4, ESC_PROTOCOL_STANDARD);
    
    // ESC_MultiCalibrate(m, 4);
    // for (int i = 0; i < 4; i++){
    //     delay(1000);
    //     ESC_Write(&m[i], 0.2f);
    //     delay(3000);
    //     ESC_Write(&m[i], 0);
    // }
    // delay(4000);
    // for (int i = 0; i < 4; i++) ESC_Write(&m[i], 0.2f);
    
    estimatorInit();
    ncInit();
    //rccomInit();
    uavcomInit();
    uavexeInit();
    
    /* SYSTEM READY FLAG */
    sysInit = 2;
    ledseqStop(LED1);
    //vec_t* pos = (vec_t*)&xkinematicsState()->position.v;
    while(1){
        //serialPrint("Position : %.2f, %.2f, %.2f\n", pos->x, pos->y, pos->z);
        delay(100);
    }
}

void systemWaitReady(void){
	while(sysInit != 2) delay(100);
}

void systemErrorCall(void){
    serialPrint("[E] System Hard Fault Error!\n");
    while(1);
}
