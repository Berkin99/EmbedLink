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
#include "system.h"
#include "systime.h"
#include "sysconfig.h"
#include "spi.h"
#include "i2c.h"
#include "uart.h"
#include "adc.h"
#include "pwm.h"
//#include "usb.h"

static uint8_t sysInit;

taskAllocateStatic(SYSTEM_TASK, SYSTEM_TASK_STACK, SYSTEM_TASK_PRI);
void systemTask(void* argv);

void systemLaunch(void){

    if(sysInit) return;
    sysInit = 1;

    spiInit();
    i2cInit();
    uartInit();
    pwmInit();
    adcInit();
    // usbInit();

    taskCreateStatic(SYSTEM_TASK, systemTask, NULL);
    taskStartScheduler();

    /* Should not reach here */
    systemErrorCall();
    while(1);
}

void systemTask(void* argv){

//    serialPrint("[>] System Start\n");
//
//    ledseqContext_t start_seq  = {LED4, 5, seq_fastblinkloop};
//    ledseqContext_t system_seq = {LED3, 5, seq_heartbeat};
//
//    ledseqInit();
//    ledseqRun (&start_seq);
//
//    delay(1000);
//
//    /* Memory First */
//    memoryInit();
//    memoryTest();
//    memoryDownload();
//
//    ncInit();
//    quadInit();
//
//    sensorInit();
//    telemetryInit();
//    navigatorInit();
//    estimatorInit();
//
//    sensorTest();
//    telemetryTest();
//    navigatorTest();
//    estimatorTest();
//
//    /* System Ready Signal */
//    ledseqStop (&start_seq);
//    ledseqRun  (&system_seq);
//
//    sysInit = 2;
//
//    serialPrint("[>] System Ready\n \n");
//    while(1){
//    	delay(1000);
//    }
}

void systemWaitReady(void){
	while(sysInit != 2) delay(100);
}

void systemErrorCall(void){
    serialPrint("[E] System Hard Fault Error!\n");
    while(1){}
}
