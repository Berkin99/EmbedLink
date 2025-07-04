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

#ifndef SENSOR_HMC5883L_H_
#define SENSOR_HMC5883L_H_

#include <stdint.h>
#include <stdbool.h>
#include "sensor.h"

#define sensorNameHMC5883L    "HMC5883L"
#define sensorFreqHMC5883L    (75)		/* Hz */

int8_t sensorInitHMC5883L(void);
int8_t sensorTestHMC5883L(void);
void   sensorCalibrateHMC5883L(void);
int8_t sensorIsCalibratedHMC5883L(void);
int8_t sensorAcquireHMC5883L(sense_t* plist, uint8_t n);
int8_t sensorIsReadyHMC5883L(void);
void   sensorWaitDataReadyHMC5883L(void);

#endif /* SENSOR_6500_H_ */
