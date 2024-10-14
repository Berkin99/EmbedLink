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

#ifdef __USBD_CDC_IF_H__

#define USB_TIMEOUT (1000)

static mutex_t usbMutex;
static semaphore_t txCplt;
static semaphore_t rxCplt;

static uint8_t* pBuffer;
static uint16_t length;

void usbInit(void){
	usbMutex = mutexCreate();
	txCplt   = semaphoreCreate();
	rxCplt   = semaphoreCreate();
}

int8_t usbReceive(uint8_t* pRxData, uint16_t len){
	semaphoreTake(rxCplt, RTOS_MAX_DELAY);
	if(len > length) len = length;
	memcpy(pRxData, pBuffer, len);
	pBuffer += len;
	length  -= len;
	return OK;
}

int8_t usbTransmit(uint8_t* pTxData, uint16_t len){
	if (mutexTake(usbMutex, USB_TIMEOUT) != RTOS_TRUE) return E_NOT_FOUND;
	CDC_Transmit_FS(pTxData, len);
	semaphoreTake(txCplt, USB_TIMEOUT);
	mutexGive(usbMutex);
	return OK;
}

uint16_t usbAvailableData(void){
	return length;
}

void usbPrint(char* format, ...){

    va_list args;
    char temp[128];
    uint8_t length = 0;

    va_start(args, format);
    vsprintf(temp,format,args);
    va_end(args);

    for(uint8_t i = 0; i < 128; i++){
    	if(temp[i] == 0x00){
    		length = i;
    		break;
    	}
    }

    usbTransmit((uint8_t*)temp, length);
}

void usbWaitDataReady(void){
	semaphoreTake(rxCplt, RTOS_MAX_DELAY);
	semaphoreGive(rxCplt);
}

void CDC_TransmitCpltCallback(void){
	semaphoreGiveISR(txCplt);
}

void CDC_ReceiveCpltCallback(uint8_t* Buf, uint32_t* Len){
	pBuffer = Buf;
	length  = (uint16_t)(*Len);
	semaphoreGiveISR(rxCplt);
}

#endif
