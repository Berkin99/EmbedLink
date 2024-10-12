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

#ifndef BATTERY_H_
#define BATTERY_H_

#include "sysdebug.h"
#include "adc.h"

typedef struct{
	uint32_t raw;
	float 	 value;
	float 	 voltage;
	ADC_HandleTypeDef* padc;
}BATT_Handle_t;

BATT_Handle_t BATT_NewHandle(ADC_HandleTypeDef* padc, float voltage);
float BATT_Voltage(BATT_Handle_t* pbatt);
float BATT_Level(BATT_Handle_t* pbatt);

#endif /* BATTERY_H_ */
