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
#include "nrx.h"
#include "navigator_zedf9p.h"
#include "ublox.h"
#include "uart.h"
#include "math3d.h"
#include "geoconfig.h"
#include "estimator.h"

static UBLOX_Handle_t gps;
static uint8_t gpsBuffer [128];

static NMEA_Location_t startLocation;

static navigation_t  loc;

static uint8_t isInit  = 0;
static uint8_t isReady = 0;
static uint8_t newData = 0;

uint16_t _navigationReadDataZEDF9P(uint8_t* pBuffer);
void     _navigatorStabilizeZEDF9P(int iter);
void     _navigatorUpdateUnitsZEDF9P(int32_t latitude, int32_t longitude);

taskAllocateStatic(ZEDF9P, NAV_TASK_STACK, NAV_TASK_PRI);
void navigatorTaskZEDF9P(void* argv);

int8_t navigatorInitZEDF9P(void){
	if(isInit) return E_OVERWRITE;
	gps = UBLOX_Init(&ZEDF9P_UART);
	taskCreateStatic(ZEDF9P, navigatorTaskZEDF9P, NULL);
	isInit = 1;
	return OK;
}

int8_t navigatorTestZEDF9P(void){return OK;}

void navigatorTaskZEDF9P(void* argv){

	delay(UBLOX_INIT_INTERVAL);
	serialSetBaudRate(&ZEDF9P_UART, BAUD_38400);		/* 1-Change The STM32 Baudrate 38400 */
														/* 2-Change The ZED F9P UART2 Baudrate 38400(Default) : [UBX-CFG-PRT] */
	UBLOX_SetBaudRate(&gps, UBLOX_BAUD_RATE_115200); 	/* 3-Change The ZED F9P UART1 Baudrate 115200 : [UBX-CFG-PRT] */
	serialSetBaudRate(&ZEDF9P_UART, BAUD_115200); 	    /* 4-Change The STM32 Baudrate 115200 */
	UBLOX_LoadConfig (&gps);						    /* 5-GLL GSA GSV RMC VTG Message Frequency to 0Hz, GGA Message Frequency to 5Hz */

	/* Start Location Stabilize */
	_navigatorStabilizeZEDF9P(100);
	serialPrint("[+] ZEDF9P Location %ld, %ld\n", startLocation.latitude, startLocation.longitude);

	loc.type = NAV_LOCATION;
	loc.location.latitude = startLocation.latitude;   /* Y axis */
	loc.location.longitude = startLocation.longitude; /* X axis */
	navigationOriginSet(&loc);

	isReady = 1;

	/* Navigator Loop */
	while(1){
		uint16_t len = _navigationReadDataZEDF9P(gpsBuffer);
		if(UBLOX_ParseNMEA(&gps, gpsBuffer, len)){
			if(gps.location.latitude != -1){
				loc.location.latitude  = gps.location.latitude;
				loc.location.longitude = gps.location.longitude;
				navigationAppend(&loc);
				newData = 1;
			}
		}
	}

	/* ERROR Notes :
	 * > Check the rover : [CFG-NAVHPG] Firmware version : 1.13
	 * > Check the rover : [UBX-RXM-RTCM] RTCM corrections receive status
	 * */
}

uint16_t _navigationReadDataZEDF9P(uint8_t* pBuffer){
	int16_t i = uartReadToIdle(gps.huart, pBuffer, 127);
    if(i <= 0) return 0;
	pBuffer[i + 1] = 0; /* Make a string */
	return i;
}

void _navigatorStabilizeZEDF9P(int iter){
	float cc = 0;
	while(cc < iter){
		int len = _navigationReadDataZEDF9P(gpsBuffer);
		if(UBLOX_ParseNMEA(&gps, gpsBuffer, (uint16_t)len)){
			if(gps.location.latitude != -1){
				cc += 1;
				startLocation.latitude  = (startLocation.latitude * (cc - 1.0f) / cc)
						                + (gps.location.latitude  / cc);
				startLocation.longitude = (startLocation.longitude * (cc - 1.0f) / cc)
						                + (gps.location.longitude  / cc);
			}
		}
	}
}

void   navigatorCalibrateZEDF9P(vec_t Correction){return;}
int8_t navigatorIsCalibratedZEDF9P(void){return 1;}
int8_t navigatorAcquireZEDF9P(navigation_t* plist, uint8_t n){return 1;}
int8_t navigatorIsReadyZEDF9P(void){return isReady;}
void   navigatorWaitDataReadyZEDF9P(void){while(!newData) delay(50);}

#endif
