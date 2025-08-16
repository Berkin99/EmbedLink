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

#ifndef UAVCOM_H_
#define UAVCOM_H_

#include "xmath3d.h"

/**
 * @brief Autnomous UAV Commander
 * 
 * > Selects the mode of the uav
 * > Operates the Arm-Takeoff-Land-Goto Commands
 * > 
 */

#define UAV_ARRIVAL_DISTANCE    (3.0f) /* Meters */
#define UAV_ARRIVAL_COUNTER_MS  (2000) /* Milliseconds */  

typedef enum{
    UAVCOM_STATE_IDLE,
    UAVCOM_STATE_READY,
    UAVCOM_STATE_AUTO,
    UAVCOM_STATE_MOVING,
    UAVCOM_STATE_TAKEOFF,
    UAVCOM_STATE_LAND,    
}uavcomState_e;

typedef enum{
    UAVCOM_CMD_ARM = 1,
    UAVCOM_CMD_DISARM,
    UAVCOM_CMD_TAKEOFF,
    UAVCOM_CMD_LAND,
    UAVCOM_CMD_MOVE,
    UAVCOM_CMD_YAW,
    UAVCOM_CMD_HOME,
    UAVCOM_CMD_KILL,
    UAVCOM_CMD_ORIGIN,
}uavcomCmd_e;

extern uavcomState_e uav_state;

void uavcomInit(void);
void uavcomTask(void* argv);
void uavcomUpdate(void);
void uavcomParse(uint8_t* data);

void uavcomArm(void);
void uavcomDisarm(void);
void uavcomTakeOff(float z);
void uavcomLand(void);
void uavcomMove(vec_t pos);
void uavcomYaw(float yaw);
void uavcomHome(void);
void uavcomKill(void);
void uavcomOrigin(uint8_t* data);

void uavcomState_IDLE(void);
void uavcomState_READY(void);
void uavcomState_AUTO(void);
void uavcomState_MOVING(void);
void uavcomState_TAKEOFF(void);
void uavcomState_LAND(void);

#endif