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

#ifndef SENSOR_BNO055_H_
#define SENSOR_BNO055_H_

#include <stdint.h>
#include <stdbool.h>
#include "sensor.h"

#define sensorNameBNO055 	"BNO055"
#define sensorFreqBNO055 	(20)		/* Hz */

int8_t sensorInitBNO055(void);
int8_t sensorTestBNO055(void);
void   sensorCalibrateBNO055(void);
int8_t sensorIsCalibratedBNO055(void);
int8_t sensorAcquireBNO055(measurement_t* plist, uint8_t n);
int8_t sensorIsReadyBNO055(void);
void   sensorWaitDataReadyBNO055(void);

#endif /* SENSOR_BNO055_H_ */
