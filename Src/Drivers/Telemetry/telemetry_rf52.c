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

#include "sysconfig.h"
#include "systime.h"

#ifdef RF52_SPI

#include "telemetry_rf52.h"
#include "rtos.h"
#include "gpio.h"
#include "uart.h"
#include "spi.h"

#define NRF52_INIT_ID       0xAA

#define NRF52_CMD_ADR_SET   0x31
#define NRF52_CMD_TX_SET    0x32
#define NRF52_CMD_CLEAR     0x33

taskAllocateStatic(RF52, TRX_TASK_STACK, TRX_TASK_PRI)
void   _telemetryTaskRF52(void* argv);
int8_t rf52_spiReceive(uint8_t* pRxData, uint16_t length);
int8_t rf52_spiTransmit(const uint8_t* pTxData, uint16_t length);

int8_t telemetryInitRF52(void){
    pinWrite(RF52_CS, HIGH);
    delay(10);

    uint8_t rxBuffer[3];
    rf52_spiReceive(rxBuffer, 3);
    serialPrint("[>] RF52 ID [0x%x][0x%x][0x%x]\n", rxBuffer[0],rxBuffer[1],rxBuffer[2]); 
    
    taskCreateStatic(RF52, _telemetryTaskRF52, NULL);
    return OK;
}

int8_t telemetryTestRF52(void){
    return OK;
}

void _telemetryTaskRF52(void* argv){

    while (1){
        delay(100);

    }
}

int8_t telemetryReceiveRF52(uint8_t* pRxData, uint16_t length){

    return OK;
}

int8_t telemetryTransmitRF52(const uint8_t* pTxData, uint16_t length){
    return OK;
}

int8_t telemetryIsReadyRF52(void){
    return 0;
}

void   telemetryWaitDataReadyRF52(void){

}

int8_t rf52_spiReceive(uint8_t* pRxData, uint16_t length){
    spiBeginTransaction(&RF52_SPI);
    pinWrite(RF52_CS, LOW);
    int8_t rslt = spiReceive(&RF52_SPI, pRxData, length);
    pinWrite(RF52_CS, HIGH);
    spiEndTransaction(&RF52_SPI);
    return rslt;
}

int8_t rf52_spiTransmit(const uint8_t* pTxData, uint16_t length){
    spiBeginTransaction(&RF52_SPI);
    pinWrite(RF52_CS, LOW);
    uint8_t pRxData[32];
    int8_t rslt = spiTransmitReceive(&RF52_SPI, pRxData, pTxData, length);
    pinWrite(RF52_CS, HIGH);
    spiEndTransaction(&RF52_SPI);
    return rslt;
}

#endif
