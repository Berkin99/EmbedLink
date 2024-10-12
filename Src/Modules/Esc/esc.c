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
#include "esc.h"

ESC_Handle_t ESC_NewHandle(pwm_t* pwm, float protocolUs){
	ESC_Handle_t temp;
	temp.pHandle = pwm;
	ESC_SetProtocol(&temp, protocolUs);
	ESC_Write(&temp, 0);
	return temp;
}

void ESC_SetProtocol(ESC_Handle_t* pEsc, float protocolUs){
	pEsc->protocolUs = protocolUs;
}

void ESC_Start(ESC_Handle_t* pEsc){
	ESC_Write(pEsc, 0);
}

void ESC_Write(ESC_Handle_t* pEsc, float value){
	pwmWrite(pEsc->pHandle, (value * pEsc->protocolUs) + pEsc->protocolUs);
}

float ESC_Read(ESC_Handle_t* pEsc){
	return (pwmRead(pEsc->pHandle) - pEsc->protocolUs) / pEsc->protocolUs;
}

void ESC_Calibrate(ESC_Handle_t* pEsc){
	ESC_Write(pEsc, 1);
	delay(ESC_CALIBRATION_TIME_MS);
	ESC_Write(pEsc, 0);
}

void ESC_MultiCalibrate(ESC_Handle_t* pEsc, uint8_t length){
	for(uint8_t i = 0; i < length; i++)	{ESC_Write(&pEsc[i], 1);}
	delay(ESC_CALIBRATION_TIME_MS);
	for(uint8_t i = 0; i < length; i++)	{ESC_Write(&pEsc[i], 0);}
}
