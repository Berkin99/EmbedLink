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

#ifndef QUADCOPTER_H_
#define QUADCOPTER_H_

#include <stdio.h>
#include "kinematics.h"
#include "esc.h"
#include "battery.h"
#include "quadmode.h"

typedef struct{
    quadmode_t mode;
    quadcmd_t  cmd;
    union{
        ESC_Handle_t motor[4];
        struct{
            ESC_Handle_t motorFR;
            ESC_Handle_t motorRR;
            ESC_Handle_t motorRL;
            ESC_Handle_t motorFL;
        };
    };
}quadcopter_t;

int8_t quadInit(void);
void   quadTask(void* argv);
void   quadCalibrate(void* argv);
void   quadHealthCheck(void);
int8_t quadSetMode(quadmode_e mode);
quadmode_e quadGetMode(void);
void   quadSetMotors(quadmotor_t cmd);
void   quadStop(void);

/**
 * @brief MODE = [MANUAL]
 * @param cpow : Total vertical thrust command [0, 1]
 * 
 * @param crange : Target rotation range:
 * 
 * @param x : Pitch target [-1, 1]
 * @param y : Roll target  [-1, 1]
 * @param z : Yaw target   [-1, 1]
 * 
 */
void quadcmd_MANUAL(float cpow, vec_t crange);

/**
 * @brief MODE = [MANUAL]
 * @param cpow : Total vertical thrust command [0, 1]
 * 
 * @param crange : Target rotation range:
 * @x : Pitch target [-1, 1]
 * @y : Roll target  [-1, 1]
 * @z : Yaw target   [-1, 1]
 * 
 */
void quadcmd_HEIGHT(float cpow, vec_t crange, float z);

/**
 * @param cpos : Target position relative the xkinematics in meters
 */
void quadcmd_AUTO(vec_t cpos, float yaw);

/**
 * @brief MODE = [RAW]
 * @param craw : Raw Motor command [0, 1]
 */
void quadcmd_RAW(float craw[4]);


#endif /* QUADCOPTER_H_ */
