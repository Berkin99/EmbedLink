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

#ifndef RC_INTERFACE_H_
#define RC_INTERFACE_H_

#include <stdint.h>

#define RC_CONNECTION_LOST_TIME_MS    600

#define RC_CH_DEADBAND                10
#define RC_CH_CENTER                  127
#define RC_CH_MAX                     255
#define RC_CH_MIN                     0

#define RC_CH_X         0
#define RC_CH_Y         1
#define RC_CH_Z         2
#define RC_CH_POWER     3
#define RC_CH_CONF      4
#define RC_CH_LENGTH    5

typedef enum{
    RC_DISARMED = 0,
    RC_SWITCH,
    RC_ARMED,
}RC_State_e;

typedef struct{
    float    value;
    struct {
        uint8_t deadband;
        uint8_t center;
        float   min;
        float   max;
    } settings;
}RC_Channel_t;

typedef struct {
    RC_State_e   state;
    uint32_t	 lastUpdate;
    union{
        struct{
            RC_Channel_t chX;
            RC_Channel_t chY;
            RC_Channel_t chZ;
            RC_Channel_t chPOWER;
            RC_Channel_t chCONF;
        };
        RC_Channel_t ch[RC_CH_LENGTH];
    };
}RC_Handle_t;

RC_Handle_t RC_NewHandle(void);
void RC_Calibrate    (RC_Handle_t* pRC);                 /* Find the bias and add to eeprom */
void RC_Update       (RC_Handle_t* pRC, uint8_t raw[5]); /* Constant Speed Update Needed */
void RC_Control      (RC_Handle_t* pRC);
void RC_Validity     (RC_Handle_t* pRC);
void RC_ChannelAlign (RC_Channel_t* ch, uint8_t raw);

#endif /* RC_INTERFACE_H_ */
