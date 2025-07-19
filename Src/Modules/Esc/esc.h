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

#ifndef ESC_H_
#define ESC_H_

#include <stdbool.h>
#include "pwm.h"

#define ESC_CALIBRATION_TIME_MS	  8000
#define	ESC_PROTOCOL_STANDARD	  1000
#define ESC_PROTOCOL_ONESHOT	  250
#define ESC_PROTOCOL_ONESHOT125   125
#define ESC_PROTOCOL_ONESHOT42    42

typedef struct {
	pwm_t* pHandle;
	float  protocolUs;
}ESC_Handle_t;

ESC_Handle_t ESC_NewHandle(pwm_t* pwm, float protocolUs);
void ESC_SetProtocol(ESC_Handle_t* pEsc, float protocolUs);
void ESC_Start(ESC_Handle_t* pEsc);

/*
 * @param pEsc  : Esc handle pointer, needs to be initalized.
 * @param Value : Esc power value [Min, Max] -> [0, 1]
 */
void ESC_Write(ESC_Handle_t* pEsc, float value);

/*
 * @param pEsc  : Esc handle pointer, needs to be initalized.
 * @return      : written esc power value [Min, Max] -> [0, 1]
 */
float ESC_Read(ESC_Handle_t* pEsc);

void ESC_Calibrate(ESC_Handle_t* pEsc);
void ESC_MultiCalibrate(ESC_Handle_t* pEsc, uint8_t length);

#endif /* ESC_H_ */
