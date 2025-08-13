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

#ifndef RCCOM_H_
#define RCCOM_H_

/**
 * @brief RCCOM Handles the mode management of the quadcopter
 *        and handles the faults of the RC. Sets the command
 *        values for the quadcopter.
 * 
 * @param crange [-1, 1] pitch, roll, yaw clockwise pozitive
 * @param cpow   [0,  1]
 */

typedef enum {
    RCCOM_STATE_IDLE   = 0,
    RCCOM_STATE_MANUAL = 1,
    RCCOM_STATE_HEIGHT = 2,
    RCCOM_STATE_NAV    = 3,
} rccomState_e;

void rccomInit(void);
void rccomTask(void* argv);
void rccomUpdate(void);

void rccomState_IDLE(void);
void rccomState_MANUAL(void);
void rccomState_HEIGHT(void);
void rccomState_NAV(void);

#endif /* RCCOM_H_ */