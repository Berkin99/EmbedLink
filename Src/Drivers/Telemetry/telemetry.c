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
#include <sysconfig.h>
#include <stdio.h>
#include "uart.h"
#include "telemetry.h"

#ifdef RF24_SPI
#include "telemetry_rf24.h"
#endif
#ifdef E32100_UART
#include "telemetry_e32100.h"
#endif

#define TRX_ADD(TELN) {\
	.Name = telemetryName##TELN,\
	.Type = telemetryType##TELN,\
	.Init = &telemetryInit##TELN,\
	.Test = &telemetryTest##TELN,\
	.Receive = &telemetryReceive##TELN,\
	.Transmit = &telemetryTransmit##TELN,\
	.IsReady = &telemetryIsReady##TELN,\
	.WaitDataReady = &telemetryWaitDataReady##TELN,},


static const telemetry_t trxList[] ={
	#ifdef RF24_SPI
		TRX_ADD(RF24)
	#endif
	#ifdef E32100_UART
		TRX_ADD(E32100)
	#endif
};

static const uint8_t trxLen = sizeof(trxList)/sizeof(telemetry_t);

void telemetryInit(void){
	for(uint8_t i = 0; i < trxLen; i++){
		if(trxList[i].Init() == OK) serialPrint("[+] Telemetry %s init OK\n", trxList[i].Name);
		else serialPrint("[-] Telemetry %s init ERROR\n", trxList[i].Name);
	}
}

void telemetryTest(void){
	for(uint8_t i = 0; i < trxLen; i++){
		if(trxList[i].Test() == OK) serialPrint("[+] Telemetry %s test OK\n",trxList[i].Name);
		else serialPrint("[-] Telemetry %s test ERROR\n",trxList[i].Name);
	}
}

int8_t telemetryIsReady(void){
	for(uint8_t i = 0; i < trxLen; i++){
		if(!trxList[i].IsReady()) return FALSE;
	}
	return TRUE;
}

int8_t telemetryGet(char* name, telemetry_t** ptelemetry){
	for(uint8_t i = 0; i<trxLen; i++){
		if(strcmp(trxList[i].Name, name) == 0){
			*ptelemetry = &trxList[i];
			return OK;
		}
	}
	return E_NOT_FOUND;
}

uint8_t telemetryGetSize(void){
	return trxLen;
}

int8_t telemetryReceive(char* name, uint8_t* pRxData, uint16_t length){
	telemetry_t* ptrx;
	if(telemetryGet(name, &ptrx) != OK) return E_NOT_FOUND;
	return ptrx->Receive(pRxData, length);
}

int8_t telemetryTransmit(char* name, const uint8_t* pTxData, uint8_t Length){
	telemetry_t* ptrx;
	if(telemetryGet(name, &ptrx) != OK) return E_NOT_FOUND;
	return ptrx->Transmit(pTxData, Length);
}
