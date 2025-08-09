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

#ifndef QUADCONTROL_H_
#define QUADCONTROL_H_

#include <stdint.h>
#include "xmath3d.h"
#include "kinematics.h"

typedef union{
    float m[4];
    struct{
        float mFR;
        float mRR;
        float mRL;
        float mFL;
    };
}quadmotor_t;

typedef struct{
    float craw[4];  /* [0, 1]  */
    float cpow;     /* [0, 1]  */
    vec_t crange;   /* [-1, 1] */
    vec_t cpos;     /* Position in meters relative the xkin origin */
    vec_t crot;     /* Rotation in euler angles */
}quadcmd_t;

typedef enum{
    QUAD_MODE_IDLE       = 0,
    QUAD_MODE_READY      = 1,
    QUAD_MODE_MANUAL     = 2,
    QUAD_MODE_HEIGHT     = 3,
    QUAD_MODE_AUTO       = 4,
    QUAD_MODE_RAW        = 5,
    QUAD_MODE_COUNT,
}quadmode_e;

typedef struct{
    quadmode_e  id;
    quadmotor_t (*modeUpdate)(quadcmd_t* pcmd);
    int8_t      (*modePermission)(quadmode_e lmode);
}quadmode_t;

void quadModeInit(void);

quadmode_t  quadMode   (quadmode_e id);

quadmotor_t quadTask_IDLE   (quadcmd_t* pcmd);
quadmotor_t quadTask_READY  (quadcmd_t* pcmd);
quadmotor_t quadTask_MANUAL (quadcmd_t* pcmd);
quadmotor_t quadTask_HEIGHT (quadcmd_t* pcmd);
quadmotor_t quadTask_AUTO   (quadcmd_t* pcmd);
quadmotor_t quadTask_RAW    (quadcmd_t* pcmd);

int8_t quadPermission_IDLE    (quadmode_e lmode);
int8_t quadPermission_READY   (quadmode_e lmode);
int8_t quadPermission_MANUAL  (quadmode_e lmode);
int8_t quadPermission_HEIGHT  (quadmode_e lmode);
int8_t quadPermission_AUTO    (quadmode_e lmode);
int8_t quadPermission_RAW     (quadmode_e lmode);

#endif /* QUADCONTROL_H_ */
