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
#include "math3d.h"
#include "kinematics.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Navigation type definitions */

#define NAV_TIMEOUT_MS    1000
#define NAV_UNAVILABLE    -1

typedef struct{
	struct{
	    int32_t latitude;     /* GCS */
	    int32_t longitude;    /* GCS */
	};
	vec_t stdDev;
}location_t;

typedef float  compass_t;	/* World Z axis angle [-180 , 180] 0 equals North */
typedef float altitude_t;   /* Altitude from Local Sea Level */
typedef uint64_t  time_t;	/* Unix Time 64 bit (seconds) */

typedef enum{
	NAV_LOCATION,
	NAV_COMPASS,
	NAV_ALTITUDE,
	NAV_TIME,
	NAV_TYPECOUNT,
}navstate_e;

typedef struct{
    location_t location;
    compass_t  compass;
    altitude_t altitude;
    time_t     time;
}navstate_t;

typedef struct{
	navstate_e type;
	union{
		location_t location;
		compass_t  compass;
		altitude_t altitude;
		time_t     time;
	};
}navigation_t;

navstate_t* navigationState (void);

navstate_t navigationOrigin(void);
void navigationOriginSet(navigation_t* pData);
void navigationPositionOrigin(vec_t position);

void navigationAppend (navigation_t* pData);
void navigationStateUpdate(navstate_t* pState);

int8_t navigationIsValid  (navstate_e index);
int8_t navigationValidity (navstate_e index, uint32_t timeout);

int8_t navigationLocation (location_t* pBuf);
int8_t navigationCompass  (compass_t* pBuf);
int8_t navigationAltitude (altitude_t* pBuf);
int8_t navigationTime     (time_t* pBuf);

void   navigationLocationUnits(location_t loc);
void   navigationLocationPos(position_t* pBuf);
void   navigationAltitudePos(float* pBuf);

int8_t navigationToMeasurement(const navigation_t* pData, measurement_t* pBuf);
float  navigationPressureToAltitude(float pressure);

#ifdef __cplusplus
}
#endif

#endif /* NAVIGATION_H_ */
