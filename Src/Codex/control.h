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

#ifndef CONTROL_H_
#define CONTROL_H_

#include "math3d.h"
#include "sysdebug.h"
#include "event.h"

#define CONTROL_TIMEOUT_MS    4000

typedef vec_t controlRange_t;       /* [-1, 1] */
typedef float controlPower_t;       /* [ 0, 1] */
typedef vec_t controlAttitude_t;    /* degrees/s */
typedef vec_t controlVelocity_t;    /* m/s */
typedef vec_t controlPosition_t;    /* meters */

typedef enum{
    CONTROL_RANGE,
    CONTROL_POWER,
    CONTROL_ATTITUDE,
    CONTROL_VELOCITY,
    CONTROL_POSITION,
	CONTROL_COUNT,
}control_e;

typedef struct{
    control_e type;
    union{
        controlRange_t    crange;
        controlPower_t    cpow;
        controlAttitude_t catt;
        controlVelocity_t cvel;
        controlPosition_t cpos;
    };
}control_t;

typedef enum{
    CONTROL_MODE_IDLE,
    CONTROL_MODE_MANUAL,
    CONTROL_MODE_AUTO,
    CONTROL_MODE_0,
    CONTROL_MODE_1,
    CONTROL_MODE_2,
    CONTROL_MODE_3,
    CONTROL_MODE_4,
    CONTROL_MODE_COUNT
}controller_e;

typedef struct{
    controller_e      mode;
    uint32_t          lastUpdate;   /* ms */
	controlRange_t    crange;       /* x y z [-1, 1] */
	controlPower_t    cpow;         /* [ 0, 1] */
	controlAttitude_t catt;         /* degrees/s */
	controlVelocity_t cvel;         /* m/s */
	controlPosition_t cpos;         /* meters */
}controller_t;

controller_t* controller(void);

int8_t controllerModeSet      (controller_e mode);
void   controllerModeCallBack (event_t callBack);
void   controllerUpdate       (control_t ctrl);
void   controllerTimeUpdate   (void);

int8_t controllerValidity   (uint32_t timeout);
int8_t controllerIsValid    (void);

vec_t  controlVec2Range(vec_t vector, float vscale);

#endif /* CONTROL_H_ */
