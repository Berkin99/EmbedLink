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

#ifndef SENSOR_H_
#define SENSOR_H_

#include <stdint.h>
#include <system.h>
#include <xmathf.h>

#define SENSE_AXIS_X    (1U << 0)
#define SENSE_AXIS_Y    (1U << 1)
#define SENSE_AXIS_Z    (1U << 2)

typedef enum{
    SENSE_POSITION,
    SENSE_ROTATION,
    SENSE_VELOCITY,
    SENSE_ACCELERATION,
    SENSE_ATTITUDE,
    SENSE_IROTATION,
    SENSE_IVELOCITY,
    SENSE_IACCELERATION,
    SENSE_IATTITUDE,
    SENSE_MAGNETIZATION,
    SENSE_PRESSURE,
    SENSE_TEMPERATURE
}sense_e;

typedef struct{
    sense_e  type;
    xv3f32_t xvec;
}sense_t;

typedef struct {
    const char* Name;
    int8_t      (*Init)(void);
    int8_t      (*Test)(void);
    void        (*Calibrate)(void);
    int8_t      (*IsCalibrated)(void);
    int8_t      (*Acquire)(sense_t* plist, uint8_t length);    /* @return plist length */
    int8_t      (*IsReady)(void);
    void        (*WaitDataReady)(void);
}sensor_t;

void    sensorInit(void);
void    sensorTest(void);
int8_t  sensorIsReady(void);
uint8_t sensorSize(void);
int8_t  sensorGet(const char* name, sensor_t** psensor);

#endif /* SENSOR_H_ */
