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

#ifndef NAVIGATION_H_
#define NAVIGATION_H_

#include <stdint.h>
#include <sysdefs.h>
#include <xmathf.h>

typedef enum{
	NAV_LOCATION,
	NAV_COMPASS,
	NAV_ALTITUDE,
	NAV_UNIXTIME,
	NAV_TYPECOUNT,
}navigation_e;

typedef struct{
	f64 latitude;
	f64 longitude;
	f32 stdDev;
	u32 timestampMs;
}location_t;   					/* Degrees as f64 */

typedef xf32_t    altitude_t;   /* Altitude from Local Sea Level */
typedef xf32_t    compass_t;	/* World Z axis angle [-180 , 180] 0 equals North */
typedef u64       unixtime_t;   /* Unix Time 64 bit (seconds) */

typedef struct{
	navigation_e type;
	union{
		location_t location;
		altitude_t altitude;
		compass_t  compass;
		unixtime_t unixtime;	
	};
}navigation_t;

typedef struct{
	location_t location;
	altitude_t altitude;
	compass_t  compass;
	unixtime_t unixtime;
}navigationState_t;

void   navigationReset(navigationState_t* self);
void   navigationSetLocation(navigationState_t* self, location_t* location);
void   navigationSetAltitude(navigationState_t* self, altitude_t* altitude);
void   navigationSetCompass(navigationState_t* self, compass_t* compass);
void   navigationSetUnixtime(navigationState_t* self, unixtime_t* unixtime);
int8_t navigationIsValid(navigationState_t* self, navigation_e idx, uint32_t timeout_ms);

void   xnavigationReset(void);
void   xnavigationGetLocation(location_t* location);
void   xnavigationGetAltitude(altitude_t* altitude);
void   xnavigationGetCompass(compass_t* compass);
void   xnavigationGetUnixtime(unixtime_t* unixtime);
void   xnavigationSetLocation(location_t* location);
void   xnavigationSetAltitude(altitude_t* altitude);
void   xnavigationSetCompass(compass_t* compass);
void   xnavigationSetUnixtime(unixtime_t* unixtime);
int8_t xnavigationIsValid(navigation_e idx, uint32_t timeout_ms);
void   xnavigationUnitLocation(const location_t* loc);
void   xnavigationGetOrigin(navigationState_t* pBuf);
void   xnavigationSetOrigin(navigationState_t navigation);
void   xnavigationCalibrateOrigin(vec_t position);
int8_t xnavigationGetPosition(vec_t* pos);

float  navigationPressureToAltitude(float pressure);

#endif /* NAVIGATION_H_ */
