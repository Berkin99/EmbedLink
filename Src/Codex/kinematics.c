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

#include <systime.h>
#include "kinematics.h"
#include "matrix.h"

static kinematicsState_t _kinematics;

void kinematicsReset(kinematicsState_t *self){
    for (size_t i = 0; i < KINV_TYPECOUNT; i++) self->kinv[i] = xvzero();
}

void kinematicsSet(kinematicsState_t *self, kinematics_e idx, kinv_t data){
    self->kinv[idx] = data;
}

kinv_t kinematicsGet(kinematicsState_t *self, kinematics_e idx){
    return self->kinv[idx];
}

int8_t kinematicIsValid(kinematicsState_t *self, kinematics_e idx, uint32_t timeout_ms){
    return xvtime(&self->kinv[idx], timeout_ms);
}

void xkinematicsReset(void){kinematicsReset(&_kinematics);}

void xkinematicsSet(kinematics_e idx, kinv_t data){kinematicsSet(&_kinematics, idx, data);}

kinv_t xkinematicsGet(kinematics_e idx){return kinematicsGet(&_kinematics, idx);}

int8_t xkinematicsIsValid(kinematics_e idx, uint32_t timeout_ms){return kinematicIsValid(&_kinematics, idx, timeout_ms);}

// vec_t kinematicsRotateFrame(vec_t v, vec_t frame){
//     matrix_t R =  mnew(3, 3);
//     matrix_t mv = mnew(3, 1);

//     mv.mx[0][0] = v.x;
//     mv.mx[1][0] = v.y;
//     mv.mx[2][0] = v.z;

//     float sx = sinf(frame.x * DEG2RAD);
//     float cx = cosf(frame.x * DEG2RAD);
//     float sy = sinf(frame.y * DEG2RAD);
//     float cy = cosf(frame.y * DEG2RAD);
//     float sz = sinf(frame.z * DEG2RAD);
//     float cz = cosf(frame.z * DEG2RAD);

//     R.mx[0][0] = cy * cz;
//     R.mx[0][1] = sx * sy * cz - cx * sz;
//     R.mx[0][2] = cx * sy * cz + sx * sz;
//     R.mx[1][0] = cy * sz;
//     R.mx[1][1] = sx * sy * sz + cx * cz;
//     R.mx[1][2] = cx * sy * sz - sx * cz;
//     R.mx[2][0] = -sy;
//     R.mx[2][1] = sx * cy;
//     R.mx[2][2] = cx * cy;

//     mv = mdot(R, mv);
//     return mkvec(mv.mx[0][0], mv.mx[1][0], mv.mx[2][0]);
// }
