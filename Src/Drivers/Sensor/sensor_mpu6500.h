/**
 *    __  __ ____ _  __ ____ ___ __  __
 *    \ \/ // __// |/ //  _// _ |\ \/ /
 *     \  // _/ /    /_/ / / __ | \  /
 *     /_//___//_/|_//___//_/ |_| /_/
 *
 *         Yeniay System Firmware
 *
 *       Copyright (C) 2024 Yeniay
 *
 * This  program  is  free software:   you
 * can  redistribute it  and/or  modify it
 * under  the  terms of  the  GNU  General
 * Public  License as  published  by   the
 * Free Software Foundation, in version 3.
 *
 * You  should  have  received  a  copy of
 * the  GNU  General  Public License along
 * with this program. If not, see
 * <http://www.gnu.org/licenses/>.
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
