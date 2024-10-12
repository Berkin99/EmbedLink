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

#ifndef KINEMATICS_H_
#define KINEMATICS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "sysdebug.h"
#include "math3d.h"

#define INDEX_X       0
#define INDEX_Y       1
#define INDEX_Z       2

#define STATE_X    (1U<<0)
#define STATE_Y    (1U<<1)
#define STATE_Z    (1U<<2)

#define STATE_XYZ  (STATE_X | STATE_Y | STATE_Z)

#define KINEMATICS_TIMEOUT_MS 1000

#define STATE_DEFINITE    -1
#define STATE_AXIS_OUT    -2

/*
 *  @brief  This  file  is referenced by
 *  all sensor measurements, it can used
 *  for estimators.
 *
 *  @param stdDev: stdDev defines the standard
 *  deviation of the measurement. It has to be
 *  positive value.
 *
 *  @stdDev =  0: not defined standard deviation.
 *  @stdDev = -1: definite correction data
 *  @stdDev = -2: defines the axis is out of
 *  usage.
 */

typedef struct{
	union{
		struct{
			float x;
			float y;
			float z;
		};
		float axis[3];
		float value;
		vec_t vector;
	};
    vec_t stdDev;
}kinv_t;

static inline kinv_t kinzero(void){
	kinv_t tmp;
	tmp.vector = vzero();
	tmp.stdDev = vzero();
	return tmp;
}

typedef kinv_t position_t;
typedef kinv_t rotation_t;
typedef kinv_t velocity_t;
typedef kinv_t acceleration_t;
typedef kinv_t attitude_t;
typedef kinv_t magnetization_t;
typedef kinv_t pressure_t;
typedef kinv_t temperature_t;

typedef enum{
    STATE_POSITION,              /* meters */
    STATE_ROTATION,              /* deg */
    STATE_VELOCITY,              /* m/s */
    STATE_ACCELERATION,          /* m/s^2 */
    STATE_ATTITUDE,              /* deg/s */
    STATE_INTERNAL_VELOCITY,     /* m/s */
    STATE_INTERNAL_ACCELERATION, /* m/s^2 */
    STATE_INTERNAL_ATTITUDE,     /* deg/s */
	STATE_MAGNETIZATION,         /* azimuth */
    STATE_PRESSURE,              /* mbar */
    STATE_TEMPERATURE,           /* celcius */

    STATE_TYPECOUNT
}state_e;

typedef union{
	struct{
		position_t      position;      /* World frame position */
		rotation_t      rotation;      /* World frame rotation */
		velocity_t      velocity;      /* World frame speed m/s */
		acceleration_t  acceleration;  /* World frame acceleration */
		attitude_t      attitude;      /* World frame angular speed */
		velocity_t      ivelocity;     /* Self frame speed m/s */
		acceleration_t  iacceleration; /* Self frame acceleration */
		attitude_t      iattitude;     /* Self frame angular speed */
		magnetization_t magnetization; /* Self frame microTesla */
        pressure_t      pressure;
        temperature_t   temperature;
	};
	kinv_t kinv[STATE_TYPECOUNT];
}state_t;

typedef struct{
    state_e type;
    union{
        position_t      position;
        rotation_t      rotation;
        velocity_t      velocity;
        acceleration_t  acceleration;
        attitude_t      attitude;
        magnetization_t magnetization;
        pressure_t      pressure;
        temperature_t   temperature;
        kinv_t			kinv;
    };
}measurement_t;

state_t* kinematicsState(void);
void     kinematicsAppend(state_e index, kinv_t data);
void     kinematicsStateUpdate(state_t* pState, uint8_t* checkList);
int8_t   kinematicsValidity(state_e index, uint32_t timeout);
int8_t   kinematicsVector(state_e index, kinv_t* pBuffer);
vec_t    kinematicsRotateFrame(vec_t v, vec_t frame);

#ifdef __cplusplus
}
#endif

#endif /* KINEMATICS_H_ */
