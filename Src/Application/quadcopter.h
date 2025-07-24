// /*
//  *       ______          __             ____    _       __
//  *      / ____/___ ___  / /_  ___  ____/ / /   (_)___  / /__
//  *     / __/ / __ `__ \/ __ \/ _ \/ __  / /   / / __ \/ //_/
//  *    / /___/ / / / / / /_/ /  __/ /_/ / /___/ / / / / ,<
//  *   /_____/_/ /_/ /_/_.___/\___/\__,_/_____/_/_/ /_/_/|_|
//  *
//  *  EmbedLink Firmware
//  *  Copyright (c) 2024 Yeniay RD, All rights reserved.
//  *  _________________________________________________________
//  *
//  *  EmbedLink Firmware is free software: you can redistribute
//  *  it and/or  modify it under  the  terms of the  GNU Lesser
//  *  General Public License as  published by the Free Software
//  *  Foundation,  either version 3 of the License, or (at your
//  *  option) any later version.
//  *
//  *  EmbedLink  Firmware is  distributed  in the  hope that it
//  *  will be useful, but  WITHOUT  ANY  WARRANTY; without even
//  *  the implied warranty of MERCHANTABILITY or FITNESS FOR A
//  *  PARTICULAR PURPOSE.  See  the GNU  Lesser  General Public
//  *  License for more details.
//  *
//  *  You should have received a copy of the GNU Lesser General
//  *  Public License along with EmbedLink Firmware. If not, see
//  *  <http://www.gnu.org/licenses/>.
//  *
//  */

// #ifndef QUADCOPTER_H_
// #define QUADCOPTER_H_

// #include <stdio.h>
// #include "kinematics.h"
// #include "esc.h"
// #include "control.h"
// #include "battery.h"
// #include "xlist.h"

// #define QUAD_TASK_STACK      (6 * configMINIMAL_STACK_SIZE)
// #define QUAD_TASK_PRI        (6)

// typedef union{
//     float m[4];
//     struct{
//         float mFR;
//         float mRR;
//         float mRL;
//         float mFL;
//     };
// }quadmotor_t;

// typedef enum{
//     QUAD_IDLE,
//     QUAD_MANUAL,
//     QUAD_HEIGHT,
//     QUAD_AUTO,
// 	QUAD_TAKEOFF,
//     QUAD_LAND,
//     QUAD_MODE_COUNT,
// }quadmode_e;

// typedef struct{
//     quadmode_e  modeid;
//     xlist_t      demand;
//     quadmotor_t (*modeUpdate)(void);
// }quadmode_t;

// typedef struct{
//     quadmode_t   mode;
//     union{
//         ESC_Handle_t motor[4];
//         struct{
//             ESC_Handle_t motorFR;
//             ESC_Handle_t motorRR;
//             ESC_Handle_t motorRL;
//             ESC_Handle_t motorFL;
//         };
//     };
// }quadcopter_t;

// void   quadInit(void);
// void   quadTask(void* argv);
// void   quadCalibrate(void* argv);
// void   quadHealthCheck(quadcopter_t* pHandle);
// int8_t quadSetMode(quadmode_e mode);
// void   quadControlModeCallBack(void);
// int8_t quadSetMotors(quadcopter_t* pHandle, quadmotor_t cmd);
// void   quadStop(quadcopter_t* pHandle);

// #endif /* QUADCOPTER_H_ */