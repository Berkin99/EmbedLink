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

#include <sysdefs.h>
#include <sysconfig.h>
#include <systime.h>
#include <string.h>
#include "rtos.h"
#include "uart.h"
#include "nrx.h"
#include "sensor_rpi2w.h"

#ifdef  RPI2W_UART

taskAllocateStatic(RPI2W, SENS_TASK_STACK, SENS_TASK_PRI);
void sensorTaskRPI2W (void* argv);

static int8_t isInit;
static int8_t isReady;
static int32_t aruco_id;
    
int8_t sensorInitRPI2W(void){
    if(isInit) return E_OVERWRITE;
    taskCreateStatic(RPI2W, sensorTaskRPI2W, NULL);
    return OK;
}

int8_t sensorTestRPI2W(void){
    return OK;
}

void sensorTaskRPI2W(void* argv){

    isReady = 1;
    delay(500);
    
    while (1)
    {
        uint8_t rxBuffer[32] = {0};
        HAL_UART_Abort(RPI2W_UART.handle);
        uartReadToIdle(&RPI2W_UART, rxBuffer, 32);
        aruco_id = atoi((char*)rxBuffer);
        delay(20);
    }
    
}

void sensorCalibrateRPI2W(void){
    return;
}

int8_t	sensorIsCalibratedRPI2W(void){
    return OK;
}

int8_t sensorAcquireRPI2W(sense_t* plist, uint8_t n){
    return E_ERROR;
}

int8_t sensorIsReadyRPI2W(void){
    return isReady;
}

void sensorWaitDataReadyRPI2W(void){while(1);}


NRX_GROUP_START(aruco)
NRX_ADD(NRX_INT32, id, &aruco_id)
NRX_GROUP_STOP (aruco)

#endif /* RPI2W_SPI */
