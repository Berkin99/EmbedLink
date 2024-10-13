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

#ifndef SENSOR_MPU6500_H_
#define SENSOR_MPU6500_H_

#include <stdint.h>
#include <stdbool.h>
#include "sensor.h"

#define sensorNameMPU6500    "MPU6500"
#define sensorFreqMPU6500    (1000)    /* Hz */

int8_t sensorInitMPU6500(void);
int8_t sensorTestMPU6500(void);
void   sensorCalibrateMPU6500(void);
int8_t sensorIsCalibratedMPU6500(void);
int8_t sensorAcquireMPU6500(measurement_t* plist, uint8_t n);
int8_t sensorIsReadyMPU6500(void);
void   sensorWaitDataReadyMPU6500(void);

#endif /* SENSOR_MPU6500_H_ */
