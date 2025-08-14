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

#include "xmath3d.h"
#include "xmath.h"
#include "pid.h"
#include "control_navigation.h"
#include "kinematics.h"
#include "filter.h"
#include "nrx.h"

/* Navigation PID */
#define PID_NAV    {3.0f, 0.0f, 12.0f} // 1.4, 0.0, 4.0
#define NAV_MAX    4.0f /* m/s */

static pid_t pidNav = PID_NAV;
static pidHandle_t hpidNav[2];

void   controlInitNAV  (void){
    /* Navigation PID Initialize */
    for (uint8_t i = 0; i < 2; i++) {
        pidInit(&hpidNav[i]);
        hpidNav[i].coefficient = pidNav;
        hpidNav[i].iLimit = 5;
        hpidNav[i].dt = 0.004f;
    }
}

/* Vector Output [-10 , 10] */
vec_t controlTaskNAV(vec_t target){
    /* World x and y vector */
    vec_t vector = vzero();

    for(uint8_t i = 0; i < 2 ; i++){
        /* Proportional */
        float error = target.axis[i] - xkinematicsState()->position.axis[i];
        vector.axis[i] = error * pidNav.kp;

        /* Derivative */
        vector.axis[i] -= xkinematicsState()->velocity.axis[i] * pidNav.kd;

        /* Constrain Max Min */
        vector.axis[i] = clampf32(vector.axis[i], -NAV_MAX, NAV_MAX);
    }

    return vector;
}

void controlResetNAV (void){
    for(uint8_t i = 0; i < 2; i++) {pidReset(&hpidNav[i]);}
}

NRX_GROUP_START(pidnav)
NRX_ADD(NRX_FLOAT, kp, &pidNav.kp)
NRX_ADD(NRX_FLOAT, ki, &pidNav.ki)
NRX_ADD(NRX_FLOAT, kd, &pidNav.kd)
NRX_GROUP_STOP(pidnav)
