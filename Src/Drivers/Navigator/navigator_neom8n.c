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

#ifdef NEOM8N_UART

#include "rtos.h"
#include "nrx.h"
#include "navigator_NEOM8N.h"
#include "ublox.h"
#include "serial.h"
#include "math3d.h"
#include "geoconfig.h"
#include "estimator.h"

static UBLOX_Handle_t gps;
static uint8_t gpsBuffer [128];
static NMEA_Location_t startLocation;
static navigation_t loc;

static uint8_t isInit  = 0;
static uint8_t isReady = 0;
static uint8_t newData = 0;

uint16_t _navigationReadDataNEOM8N(uint8_t* pBuffer);
void    _navigatorStabilizeNEOM8N(int iter);

STATIC_MEM_TASK_ALLOC(NEOM8N,NAV_TASK_STACK,NAV_TASK_PRI);
void navigatorTaskNEOM8N(void* argv);

int8_t navigatorInitNEOM8N(void){
	if(isInit) return SYS_E_OVERWRITE;
	gps = UBLOX_Init(&NEOM8N_UART);
	STATIC_MEM_TASK_CREATE(NEOM8N, navigatorTaskNEOM8N, NULL);
	isInit = 1;
	return SYS_OK;
}

int8_t navigatorTestNEOM8N(void){return SYS_OK;}

void navigatorTaskNEOM8N(void* argv){

	delay(UBLOX_INIT_INTERVAL);
	UBLOX_LoadConfig(&gps);

	_navigatorStabilizeNEOM8N(100);
	serialPrint("[+] NEOM8N Location %ld, %ld\n", startLocation.latitude, startLocation.longitude);

	loc.type = NAV_LOCATION;
	loc.location.latitude = startLocation.latitude;   /* Y axis */
	loc.location.longitude = startLocation.longitude; /* X axis */
	navigationOriginSet(&loc);

	isReady = 1;

	while(1){
		uint8_t len = _navigationReadDataNEOM8N(gpsBuffer); /* Serial Read to idle (No need to RTOS delay)*/
		if(UBLOX_ParseNMEA(&gps, gpsBuffer, len)){
			if(gps.location.latitude != -1){
				loc.location.latitude  = gps.location.latitude;
				loc.location.longitude = gps.location.longitude;
				navigationAppend(&loc);
				newData = 1;
			}
		}
	}
}

uint16_t _navigationReadDataNEOM8N(uint8_t* pBuffer){
	int16_t i = serialReadToIdle(gps.huart, pBuffer, 127);
    if(i <= 0) return 0;
	pBuffer[i + 1] = 0; /* Make a string */
	return i;
}

void _navigatorStabilizeNEOM8N(int iter){
	float cc = 0;
	while(cc < iter){
		int len = _navigationReadDataNEOM8N(gpsBuffer);
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

void   navigatorCalibrateNEOM8N(vec_t Correction){return;}
int8_t navigatorIsCalibratedNEOM8N(void){return SYS_TRUE;}
int8_t navigatorAcquireNEOM8N(navigation_t* plist, uint8_t n){return 0;}
int8_t navigatorIsReadyNEOM8N(void){return isReady;}
void   navigatorWaitDataReadyNEOM8N(void){while(newData == 0){delay(5);}}

#endif
