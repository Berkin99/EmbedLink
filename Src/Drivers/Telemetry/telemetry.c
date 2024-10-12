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


static const TRX_Handle_t trxList[] ={
	#ifdef RF24_SPI
		TRX_ADD(RF24)
	#endif
	#ifdef E32100_UART
		TRX_ADD(E32100)
	#endif
};

static const uint8_t trxLen = sizeof(trxList)/sizeof(TRX_Handle_t);

void telemetryInit(void){
	for(uint8_t i = 0; i < trxLen; i++){
		if(trxList[i].Init() == OK) serialPrint("[+] Telemetry %s init OK\n", trxList[i].Name);
		else serialPrint("[-] Telemetry %s init ERROR\n", trxList[i].Name);
	}
}

void telemetryTest(void){
	for(uint8_t i = 0; i<trxLen; i++){
		if(trxList[i].Test() == OK) serialPrint("[+] Telemetry %s test OK\n",trxList[i].Name);
		else serialPrint("[-] Telemetry %s test ERROR\n",trxList[i].Name);
	}
}

int8_t telemetryIsReady(uint8_t index){
	if(index >= trxLen) return E_OVERFLOW;
	return trxList[index].IsReady();
}

void telemetryWaitDataReady(uint8_t index){
	if(index >= trxLen) return;
	trxList[index].WaitDataReady();
}

int8_t telemetryGetIndex(char* name){
	for(uint8_t i = 0; i<trxLen; i++){
		if(strcmp(trxList[i].Name, name) == 0) return i;
	}
	return -1;
}

int8_t telemetryGetSize(void){
	return trxLen;
}

int8_t telemetryReceive(uint8_t index, uint8_t* pRxBuffer){
	if(index >= trxLen) return E_OVERFLOW;
	return trxList[index].Receive(pRxBuffer);
}

int8_t telemetryTransmit(uint8_t index, const uint8_t* pTxData, uint8_t Length){
	if(index >= trxLen) return E_OVERFLOW;
	return trxList[index].Transmit(pTxData, Length);
}
