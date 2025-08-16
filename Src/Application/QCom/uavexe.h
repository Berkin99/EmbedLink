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

#ifndef UAVEXE_H_
#define UAVEXE_H_

#include <stdint.h>

#define UAVEXE_CMD_P_LEN (18)
#define UAVEXE_FID_P_LEN (17)

/**
 *  @brief CMD_P   : [CMD_ID, FID_P <17 byte>]    : 18byte
 *  @brief FID_P   : {FID_ID, argument <16 byte>] : 17byte
 */

typedef enum{
    UAVEXE_CMD_PARSE,
    UAVEXE_CMD_SET,
    UAVEXE_CMD_LAUNCH,
}uavexe_cmd_e;

typedef enum{
    UAVEXE_FID_DELAY,
    UAVEXE_FID_UAVCMD,
    UAVEXE_FID_PRINT,
}uavexe_fid_e;

void uavexeInit(void);
void uavexeTask(void* argv);
void uavexeCmdParse(uint8_t* data);

void uavexeFidParse(uint8_t* data);
void uavexeFidSet(uint8_t* data);

void uavexeLaunch(uint8_t* data);

#endif