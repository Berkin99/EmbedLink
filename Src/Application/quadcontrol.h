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
#include "quadcopter.h"

void        quadControlInit (void);
quadmode_t  quadMode(quadmode_e modeid);

quadmotor_t quadIdle         (void);
quadmotor_t quadReady        (void);
quadmotor_t quadManual       (void);
quadmotor_t quadManualHeight (void);
quadmotor_t quadAutoNav      (void);
quadmotor_t quadTakeOff      (void);
quadmotor_t quadLand         (void);

#endif /* QUADCONTROL_H_ */
