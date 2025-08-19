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
#include "system.h"

#ifdef ZEDF9P_UART

#include "rtos.h"
#include "navigator_zedf9p.h"
#include "ublox.h"
#include "nmea.h"
#include "uart.h"
#include "xmath3d.h"
#include "geoconfig.h"
#include "estimator.h"
#include "xmath.h"

static UBLOX_Handle_t  gps;
static uint8_t 		   gpsBuffer [128];
static location_t      gpsLocation;

static uint8_t isInit  = 0;
static uint8_t isReady = 0;
static uint8_t newData = 0;

uint16_t _navigationReadDataZEDF9P(uint8_t* pBuffer);
int8_t   _navigatorParseLocationZEDF9P(uint8_t* pBuffer, location_t* pLocation);
int8_t   _navigatorStabilizeZEDF9P(int iter, location_t* pLocation);
int8_t   _ubxUartWrite(void* intf, uint8_t* pTxData, uint16_t len);
int8_t   _ubxUartRead(void* intf, uint8_t* pRxData, uint16_t len);

taskAllocateStatic(ZEDF9P, NAV_TASK_STACK, NAV_TASK_PRI);
void navigatorTaskZEDF9P(void* argv);

int8_t navigatorInitZEDF9P(void){
	if(isInit) return E_OVERWRITE;
	gps = UBLOX_Init((void*)&ZEDF9P_UART, _ubxUartWrite, _ubxUartRead, delay);
	taskCreateStatic(ZEDF9P, navigatorTaskZEDF9P, NULL);
	isInit = 1;
	return OK;
}

int8_t navigatorTestZEDF9P(void){return OK;}

void navigatorTaskZEDF9P(void* argv){

	delay(UBLOX_INIT_INTERVAL);

	uartSetBaudRate(&ZEDF9P_UART, 38400); 				/* 1-Change The MCU Baudrate 38400 */
														/* 2-Change The ZED F9P UART2 Baudrate 38400(Default) : [UBX-CFG-PRT] */
	UBLOX_SetBaudRate(&gps, UBLOX_BAUD_RATE_115200); 	/* 3-Change The ZED F9P UART1 Baudrate 115200 : [UBX-CFG-PRT] */
	uartSetBaudRate(&ZEDF9P_UART, 115200);	 	 	    /* 4-Change The STM32 Baudrate 115200 */
	UBLOX_LoadConfig (&gps);						    /* 5-GLL GSA GSV RMC VTG Message Frequency to 0Hz, GGA Message Frequency to 5Hz */

	serialPrint("[+] ZEDF9P Stabilize\n ");

	/* Start Location Stabilize */
	location_t sloc = {0};
	_navigatorStabilizeZEDF9P(100, &sloc);
	sloc.timestampMs = millis();

	/* Navigation Origin Set */
	xnavigationOrigin()->location = sloc;
	
	isReady = 1;
	serialPrint("\n[+] ZEDF9P Location %.7f, %.7f\n", sloc.latitude, sloc.longitude);

	/* Navigator Loop */
	while(1){
		_navigationReadDataZEDF9P(gpsBuffer);
		int8_t status = _navigatorParseLocationZEDF9P(gpsBuffer, &gpsLocation);
		if(status <= 0) continue;
		xnavigationSetLocation(gpsLocation);
	}

	/* ERROR Notes :
	 * > Check the rover : [CFG-NAVHPG] Firmware version : 1.13
	 * > Check the rover : [UBX-RXM-RTCM] RTCM corrections receive status
	 * */
}

void   navigatorCalibrateZEDF9P(vec_t Correction){return;}
int8_t navigatorIsCalibratedZEDF9P(void){return 1;}
int8_t navigatorIsReadyZEDF9P(void){return isReady;}
void   navigatorWaitDataReadyZEDF9P(void){while(!newData) delay(50);}

uint16_t _navigationReadDataZEDF9P(uint8_t* pBuffer){
	int16_t i = uartReadToIdle(&ZEDF9P_UART, pBuffer, 127);
	//serialPrint("%s\n", pBuffer);
    if(i <= 0) return 0;
	pBuffer[i + 1] = 0; /* Make a string */
	return i;
}

/**
 * @brief Returns the quality indicator of the GPS fix.
 * 
 * @retval 0  No Fix (Invalid)             - No positioning, fix not available.
 * @retval 1  GPS Fix (Autonomous)         - Standard GPS positioning.
 * @retval 2  DGPS Fix                     - Differential GPS positioning.
 * @retval 3  PPS Fix                      - Precise Positioning Service fix.
 * @retval 4  RTK Fixed                    - Real Time Kinematic, fixed solution (high accuracy).
 * @retval 5  RTK Float                    - Real Time Kinematic, float solution (medium accuracy).
 * 
 */
int8_t _navigatorParseLocationZEDF9P(uint8_t* pBuffer, location_t* pLocation){
	/* Slice each sentence by "$" ... "<CR><LF>" */
	NMEA_Message_t nmea_msg;

	if(pBuffer[0] !='$') return 0;
	if(!NMEA_Pack(&nmea_msg,  pBuffer)) return 0;

	switch(nmea_msg.payloadId){
		case NMEA_MSG_GGA:{
		NMEA_Payload_GGA_t temp;
			if(NMEA_Parse_GGA(&temp, &nmea_msg)){
				pLocation->latitude  = (f64)temp.location.latitude  / (f64)10000000.0;
				pLocation->longitude = (f64)temp.location.longitude / (f64)10000000.0;
				return (int8_t)temp.quality;
			}
		}break;
		/* CASE */
		default:break;
	}
	return 0;
}

int8_t _navigatorStabilizeZEDF9P(int iter, location_t* pLocation){
	
	int i = 0;
	location_t stabilized;
	stabilized.latitude = 0;
	stabilized.longitude = 0;
	
	while(i < iter){
		location_t temp;
		_navigationReadDataZEDF9P(gpsBuffer);
		int8_t status = _navigatorParseLocationZEDF9P(gpsBuffer, &temp);
		
		if(status > 0){
			stabilized.latitude = meanf64(stabilized.latitude, temp.latitude, i);
			stabilized.longitude = meanf64(stabilized.longitude, temp.longitude, i);
			i++;
		}
		delay(100);
	}

	*pLocation = stabilized;

	return 1;
}

int8_t _ubxUartWrite(void* intf, uint8_t* pTxData, uint16_t len){
	return uartWrite((uart_t*)intf, pTxData, len);
}

int8_t _ubxUartRead(void* intf, uint8_t* pRxData, uint16_t len){
	return uartReadToIdle((uart_t*)intf, pRxData, len);
}

#endif