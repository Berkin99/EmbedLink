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

#ifndef BATTERY_H_
#define BATTERY_H_

#include <stdint.h>
#include <sysdefs.h>
#include "adc.h"

typedef struct{
	adc_t* 	 padc;
	float 	 voltage;
}BATT_Handle_t;

/*
 * @param padc    : Analog to digital converter handle pointer.
 * @param voltage : Max ADC value reference voltage.
 */
BATT_Handle_t BATT_NewHandle(adc_t* padc, float voltage);
float BATT_ReadLevel(BATT_Handle_t* pbatt);
float BATT_ReadVoltage(BATT_Handle_t* pbatt);

#endif /* BATTERY_H_ */
