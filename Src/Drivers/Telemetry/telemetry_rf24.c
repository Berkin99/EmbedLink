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
#include "sysconfig.h"
#include "systime.h"
#include "rtos.h"

#ifdef RF24_SPI
#include "telemetry_rf24.h"
#include "RF24.h"
#include "event.h"
#include "uart.h"

#define RF24_BUFFER_LEN 	32

typedef struct{
	uint8_t size;
	uint8_t buffer[32];
}rfData_t;

static uint8_t 	rxBuffer[RF24_BUFFER_LEN];
static rfData_t txBuffer;

static uint8_t rxaddress[] = RF24_RX_ADDRESS;
static uint8_t txaddress[] = RF24_TX_ADDRESS;

static uint8_t isReady = 0;
static uint8_t newData = 0;
static uint8_t isInit  = 0;

static semaphore_t rxSemaphore;
static semaphore_t txMutex;
static queue_t txQueue;

static uint8_t isListening = 0;

taskAllocateStatic(RF24,TRX_TASK_STACK,TRX_TASK_PRI)
queueAllocateStatic(txQueue, 6, sizeof(rfData_t))

void _telemetryTaskRF24(void* argv);

int8_t telemetryInitRF24(void){
	if (isInit == 1) return E_OVERWRITE;

	RF24_Init(&RF24_SPI, RF24_CE, RF24_CS);
	if(!RF24_begin()) return ERROR;

	rxSemaphore = semaphoreCreate();
	txMutex 	= mutexCreate();
	txQueue     = queueCreateStatic(txQueue);

	taskCreateStatic(RF24, _telemetryTaskRF24, NULL);

	isInit = 1;
	return OK;
}

int8_t telemetryTestRF24(void){
	return RF24_available();
}

void _telemetryTaskRF24(void* argv){

	RF24_setDataRate(RF24_2MBPS);
	RF24_setPALevel(RF24_PA_MAX, 0);
	RF24_openWritingPipe(txaddress); 	/*Always uses Pipe 0*/ /*301*/
	RF24_openReadingPipe(1, rxaddress);					   	   /*300*/
	RF24_startListening();

	delay(1);
	isReady = TRUE;
	uint32_t lastWakeTime = taskGetTickCount();

	while(1){

		if (!isListening){RF24_startListening(); isListening = 1;}

		if (RF24_available()){
			RF24_read(rxBuffer, RF24_BUFFER_LEN);
			newData = 1;
			xSemaphoreGive(rxSemaphore);
		}

		while (queueReceive(txQueue, &txBuffer, 0) == pdPASS){
			if(isListening){RF24_stopListening();isListening = 0;}
			RF24_write(txBuffer.buffer, txBuffer.size);
		}

		taskDelayUntil(&lastWakeTime, 4);
	}

}

int8_t telemetryReceiveRF24(uint8_t *pRxBuffer){

	if(!newData) return 0;

	for(uint8_t i = 0; i < RF24_BUFFER_LEN; i++){
		pRxBuffer[i] = rxBuffer[i];
	}

	newData = 0;

	return RF24_BUFFER_LEN;
}

int8_t telemetryTransmitRF24(const uint8_t *pTxData, uint8_t Length){
	if(!isInit) return E_CONF_FAIL;
	if(pTxData == NULL) return E_NULL_PTR;
	if(Length > RF24_MAX_PAYLOAD_LENGHT) return E_OVERFLOW;

	rfData_t temp;
	temp.size = Length;

	memcpy(temp.buffer, pTxData, Length);

	if (queueSend(txQueue, &temp, 0) == pdPASS) return OK;

	return ERROR;
}

int8_t telemetryIsReadyRF24(void){
	return isReady;
}

void telemetryWaitDataReadyRF24(void){
	xSemaphoreTake(rxSemaphore,portMAX_DELAY);
}

#endif
