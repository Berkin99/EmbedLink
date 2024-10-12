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

#ifndef SENSOR_BMP388_H_
#define SENSOR_BMP388_H_

#include <stdint.h>
#include <stdbool.h>
#include "sensor.h"

#define sensorNameBMP388    "BMP388"
#define sensorFreqBMP388    (100)        /* Hz */

int8_t sensorInitBMP388(void);
int8_t sensorTestBMP388(void);
void   sensorCalibrateBMP388(void);
int8_t sensorIsCalibratedBMP388(void);
int8_t sensorAcquireBMP388(measurement_t* plist, uint8_t n);
int8_t sensorIsReadyBMP388(void);
void   sensorWaitDataReadyBMP388(void);

#endif /* SENSOR_BMP388_H_ */
