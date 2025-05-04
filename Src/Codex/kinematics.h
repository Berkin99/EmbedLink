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

 /**
 *  @brief  This  file  is referenced by
 *  all sensor measurements, it can used
 *  for estimators.
 *
 *  @param stdDev: stdDev defines the standard
 *  deviation of the measurement. It has to be
 *  positive value.
 *
 *  @stdDev =  0: definite correction data
 *  @stdDev = -1: not defined standard deviation.
 *  @stdDev = -2: invalid standart deviation
 */

#ifndef KINEMATICS_H_
#define KINEMATICS_H_

#include <stdint.h>
#include <sysdefs.h>
#include <xmathf.h>

typedef enum{
    KINV_POSITION,              /* meters */
    KINV_ROTATION,              /* deg */
    KINV_VELOCITY,              /* m/s */
    KINV_ACCELERATION,          /* m/s^2 */
    KINV_ATTITUDE,              /* deg/s */
    KINV_IROTATION,             /* deg */
    KINV_IVELOCITY,             /* m/s */
    KINV_IACCELERATION,         /* m/s^2 */
    KINV_IATTITUDE,             /* deg/s */
    KINV_TYPECOUNT
}kinematics_e;

typedef xv3f32_t kinv_t;

typedef kinv_t position_t;
typedef kinv_t rotation_t;
typedef kinv_t velocity_t;
typedef kinv_t acceleration_t;
typedef kinv_t attitude_t;

typedef union{
    struct
    {
        position_t position;      		/* Position m 	 */
        rotation_t rotation;      		/* Rotation deg  */
        velocity_t velocity;      		/* Speed m/s 	 */
        acceleration_t acceleration;  	/* Acceleration  */
        attitude_t attitude;      		/* Angular speed */
        rotation_t irotation;      		/* Rotation deg  */
        velocity_t ivelocity;      		/* Speed m/s 	 */
        acceleration_t iacceleration;  	/* Acceleration  */
        attitude_t iattitude;      		/* Angular speed */
    };
    kinv_t kinv[KINV_TYPECOUNT];
}kinematicsState_t;

void   kinematicsReset(kinematicsState_t *self);
kinv_t kinematicsGet(kinematicsState_t *self, kinematics_e idx);
void   kinematicsSet(kinematicsState_t *self, kinematics_e idx, kinv_t data);
int8_t kinematicsIsValid(kinematicsState_t *self, kinematics_e idx, uint32_t timeout_ms);

void   xkinematicsReset(void);
kinv_t xkinematicsGet(kinematics_e idx);
void   xkinematicsSet(kinematics_e idx, kinv_t data);
int8_t xkinematicsIsValid(kinematics_e idx, uint32_t timeout_ms);
const  kinematicsState_t* xkinematicsState(void);

#endif /* KINEMATICS_H_ */
