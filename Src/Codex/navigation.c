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
#include <systime.h>
#include "navigation.h"
#include "geoconfig.h"
#include "nrx.h"

#define BASE_LATITUDE  (111000.0)
#define BASE_LONGITUDE (2 * M_PI_F64 * EARTH_EQX_R / 360.0)

static f64 UNITLAT = BASE_LATITUDE;   // Y axis meters
static f64 UNITLON = BASE_LONGITUDE;  // X axis meters

static navigationState_t _navigation;
static navigationState_t _origin;

void   navigationReset(navigationState_t* self){
	self->location = (location_t){0,0,0,0};
	self->compass  = xfzero();
	self->altitude = xfzero();
	self->unixtime = 0;
}

void   navigationSetLocation(navigationState_t* self, location_t* location){
	self->location = *location;
	self->location.timestampMs = millis();
}

void   navigationSetAltitude(navigationState_t* self, altitude_t* altitude){
	self->altitude = *altitude;
	self->altitude.timestampMs = millis();
}

void   navigationSetCompass(navigationState_t* self, compass_t* compass){
	self->compass = *compass;
	self->compass.timestampMs = millis();
}

void   navigationSetUnixtime(navigationState_t* self, unixtime_t* unixtime){
	self->unixtime = *unixtime;
}

int8_t navigationIsValid(navigationState_t* self, navigation_e idx, uint32_t timeout_ms){
	switch (idx){
		case NAV_LOCATION:	return (self->location.timestampMs + timeout_ms < millis());
		case NAV_ALTITUDE:	return (self->altitude.timestampMs + timeout_ms < millis());
		case NAV_COMPASS:	return (self->compass.timestampMs  + timeout_ms < millis());
	}
	return E_NOT_FOUND;
}

void   xnavigationReset(void){navigationReset(&_navigation);}
void   xnavigationGetLocation(location_t* location){*location = _navigation.location;}
void   xnavigationGetAltitude(altitude_t* altitude){*altitude = _navigation.altitude;}
void   xnavigationGetCompass(compass_t* compass){*compass = _navigation.compass;}
void   xnavigationGetUnixtime(unixtime_t* unixtime){*unixtime = _navigation.unixtime;}
void   xnavigationSetLocation(location_t* location){navigationSetLocation(&_navigation, location);}
void   xnavigationSetAltitude(altitude_t* altitude){navigationSetAltitude(&_navigation, altitude);}
void   xnavigationSetCompass(compass_t* compass){navigationSetCompass(&_navigation, compass);}
void   xnavigationSetUnixtime(unixtime_t* unixtime){navigationSetUnixtime(&_navigation, unixtime);}
int8_t xnavigationIsValid(navigation_e idx, uint32_t timeout_ms){navigationIsValid(&_navigation, idx, timeout_ms);}

void xnavigationUnitLocation(const location_t* loc){
	UNITLON = (2.0) * (M_PI_F64) * (EARTH_EQX_R * cos(loc->latitude * DEG2RAD))  / (360.0); /* (2.pi.r / 360) */
}

void xnavigationGetOrigin(navigationState_t* pBuf){
	*pBuf = _origin;
}

void xnavigationSetOrigin(navigationState_t navigation){
	_origin = navigation;
}

void xnavigationCalibrateOrigin(vec_t position){
	location_t temp = _navigation.location;
	temp.latitude  -= (position.y / UNITLAT);
	temp.longitude -= (position.x / UNITLON);
	_origin.location = temp;
	_origin.altitude.value = _navigation.altitude.value - position.z;
}

int8_t xnavigationGetPosition(vec_t* pos){
	location_t ldif;
	ldif.latitude  = _navigation.location.latitude  - _origin.location.latitude;
	ldif.longitude = _navigation.location.longitude - _origin.location.longitude;
	pos->y = (float)(ldif.latitude  * UNITLAT); /* Y axis */
	pos->x = (float)(ldif.longitude * UNITLON); /* X axis */
	pos->z = (float)_navigation.altitude.value - _origin.altitude.value;
	return 1;
}

/* Converts pressure to altitude above sea level (ASL) in meters */
float navigationPressureToAltitude(float pressure /*, float temperature */){
	if (pressure > 0){return ((powf((CONST_SEA_PRESSURE / pressure), CONST_PF) - 1.0f) * (FIX_TEMP + 273.15f)) / 0.0065f;}
	return 0;
}