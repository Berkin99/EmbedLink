/**
 *    __  __ ____ _  __ ____ ___ __  __
 *    \ \/ // __// |/ //  _// _ |\ \/ /
 *     \  // _/ /    /_/ / / __ | \  /
 *     /_//___//_/|_//___//_/ |_| /_/
 *
 *         Yeniay System Firmware
 *
 *       Copyright (C) 2024 Yeniay
 *
 * This  program  is  free software:   you
 * can  redistribute it  and/or  modify it
 * under  the  terms of  the  GNU  General
 * Public  License as  published  by   the
 * Free Software Foundation, in version 3.
 *
 * You  should  have  received  a  copy of
 * the  GNU  General  Public License along
 * with this program. If not, see
 * <http://www.gnu.org/licenses/>.
 */

#include "systime.h"
#include "battery.h"
#include "adc.h"

BATT_Handle_t BATT_NewHandle(ADC_HandleTypeDef* padc, float voltage){
	BATT_Handle_t bat;
	bat.padc = padc;
	bat.raw = 0;
	bat.value = 0;
	bat.voltage = voltage;
	return bat;
}

float BATT_Voltage(BATT_Handle_t* pbatt){
	uint32_t raw;
	if (adcRead(pbatt->padc, &raw) != HAL_OK) return -1.0f;
	pbatt->raw = raw;
	return (float)raw / 65536.0f;
}

float BATT_Level(BATT_Handle_t* pbatt){
	uint32_t raw;
	if (adcRead(pbatt->padc, &raw) != HAL_OK) return -1.0f;
	pbatt->raw = raw;
	return (float)raw / 65536.0f;
}
