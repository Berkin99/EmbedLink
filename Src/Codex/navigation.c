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

#include <math.h>
#include "systime.h"
#include "navigation.h"
#include "geoconfig.h"
#include "nrx.h"

static float LONLSB = 0.0111f; /* meters */
static float LATLSB = 0.0111f; /* meters */

static navstate_t navstate;
static navstate_t origin;

static uint32_t navstateTimestamps[NAV_TYPECOUNT];

navstate_t* navigationState (void){
	return &navstate;
}

navstate_t navigationOrigin(void){
	return origin;
}

void navigationOriginSet(navigation_t* pData){
	switch (pData->type) {
		case NAV_LOCATION:{
			origin.location       = pData->location;
			navigationLocationUnits(origin.location);
			break;
		}
		case NAV_COMPASS:        origin.compass        = pData->compass;  break;
		case NAV_ALTITUDE:       origin.altitude       = pData->altitude; break;
		case NAV_TIME:           origin.time           = pData->time;     break;
		default: return; break;
	}
}

void navigationPositionOrigin(vec_t position){
	location_t temp = navstate.location;
	temp.latitude  -= (int32_t)(position.y / LATLSB);
	temp.longitude -= (int32_t)(position.x / LONLSB);
	origin.location = temp;
	origin.altitude = navstate.altitude;
}

void navigationAppend (navigation_t* pData){
	switch (pData->type) {
        case NAV_LOCATION:       navstate.location     = pData->location; break;
        case NAV_COMPASS:        navstate.compass      = pData->compass;  break;
        case NAV_ALTITUDE:       navstate.altitude     = pData->altitude; break;
        case NAV_TIME:           navstate.time         = pData->time;     break;
        default: return; break;
    }
    navstateTimestamps[pData->type] = millis();
}

void navigationStateUpdate (navstate_t* pState){
	origin = *pState;
}

int8_t navigationIsValid (navstate_e index){
	return navigationValidity(index, NAV_TIMEOUT_MS);
}

int8_t navigationValidity (navstate_e index, uint32_t timeout){
	if(millis() - navstateTimestamps[index] > timeout) return FALSE;
	return TRUE;
}

int8_t navigationLocation(location_t* pBuf){
    *pBuf = navstate.location;
	if(navigationIsValid(NAV_LOCATION)){return TRUE;}
	return FALSE;
}

int8_t navigationCompass(compass_t* pBuf){
	*pBuf = navstate.compass;
	if(navigationIsValid(NAV_COMPASS)){
    	return TRUE;
	}
	return FALSE;
}

int8_t navigationAltitude(altitude_t* pBuf){
	*pBuf = navstate.altitude;
	if(navigationIsValid(NAV_ALTITUDE)){
    	return TRUE;
	}
	return FALSE;
}

int8_t navigationTime(time_t* pBuf){
    *pBuf = navstate.time;
	if(navigationIsValid(NAV_TIME)){
	    return TRUE;
	}
	return FALSE;
}

void navigationLocationUnits(location_t loc){
	double lat = (double)(loc.latitude) / 10000000.0; /* latitude as degrees */
	double ulon = (2.0) * (M_PI_F) * (EARTH_EQX_R * cos(lat * DEG2RAD))  / (360.0); /* (2.pi.r / 360) */
	LONLSB = (float)(ulon / 10000000.0f); /* degree meter to lsb meter */
}

void navigationLocationPos(position_t* pBuf){
	location_t ldif;
	ldif.latitude  = navstate.location.latitude  - origin.location.latitude;
	ldif.longitude = navstate.location.longitude - origin.location.longitude;

	pBuf->y = (float)ldif.latitude  * LATLSB; /* Y axis */
	pBuf->x = (float)ldif.longitude * LONLSB; /* X axis */
}

void   navigationAltitudePos(float* pBuf){
	*pBuf = navstate.altitude - origin.altitude;
}

/* Converts pressure to altitude above sea level (ASL) in meters */
float navigationPressureToAltitude(float pressure /* float temperature */){
	if (pressure > 0){return ((powf((CONST_SEA_PRESSURE / pressure), CONST_PF) - 1.0f) * (FIX_TEMP + 273.15f)) / 0.0065f;}
	return 0;
}

NRX_GROUP_START(location)
NRX_ADD(NRX_INT32, lat, &navstate.location.latitude)
NRX_ADD(NRX_INT32, lon, &navstate.location.longitude)
NRX_GROUP_STOP(location)

NRX_GROUP_START(altitude)
NRX_ADD(NRX_FLOAT, value, &navstate.altitude)
NRX_GROUP_STOP(altitude)

NRX_GROUP_START(compass)
NRX_ADD(NRX_FLOAT, value, &navstate.compass)
NRX_GROUP_STOP(compass)
