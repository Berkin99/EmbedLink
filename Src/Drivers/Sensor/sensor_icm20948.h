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

#ifndef SENSOR_ICM20948_H_
#define SENSOR_ICM20948_H_

#include <stdint.h>
#include <stdbool.h>
#include "sensor.h"

#define sensorNameICM20948    "ICM20948"
#define sensorFreqICM20948    (75)		/* Hz */

int8_t sensorInitICM20948(void);
int8_t sensorTestICM20948(void);
void   sensorCalibrateICM20948(void);
int8_t sensorIsCalibratedICM20948(void);
int8_t sensorAcquireICM20948(sense_t* plist, uint8_t n);
int8_t sensorIsReadyICM20948(void);
void   sensorWaitDataReadyICM20948(void);

#endif /* SENSOR_ICM20948_H_ */
