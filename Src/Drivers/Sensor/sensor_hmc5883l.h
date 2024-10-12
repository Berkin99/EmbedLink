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
int8_t sensorAcquireHMC5883L(measurement_t* plist, uint8_t n);
int8_t sensorIsReadyHMC5883L(void);
void   sensorWaitDataReadyHMC5883L(void);

#endif /* SENSOR_6500_H_ */
