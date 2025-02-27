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

#ifndef SENSOR_BMP581_H_
#define SENSOR_BMP581_H_

#include <stdint.h>
#include <stdbool.h>
#include "sensor.h"

#define sensorNameBMP581    "BMP581"
#define sensorFreqBMP581    (100)        /* Hz */

int8_t sensorInitBMP581(void);
int8_t sensorTestBMP581(void);
void   sensorCalibrateBMP581(void);
int8_t sensorIsCalibratedBMP581(void);
int8_t sensorAcquireBMP581(measurement_t* plist, uint8_t n);
int8_t sensorIsReadyBMP581(void);
void   sensorWaitDataReadyBMP581(void);

#endif /* SENSOR_BMP581_H_ */
