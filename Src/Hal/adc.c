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

#include "system.h"
#include "sysconfig.h"
#include "adc.h"
#include "FreeRTOS.h"
#include "semphr.h"

#ifdef HAL_ADC_MODULE_ENABLED

#define ADC_TIMEOUT	(1000)

typedef enum{
#ifdef ADC1_HANDLE
	ADC1_INDEX,
#endif
#ifdef ADC2_HANDLE
	ADC2_INDEX,
#endif
#ifdef ADC3_HANDLE
	ADC3_INDEX,
#endif
#ifdef ADC4_HANDLE
	ADC4_INDEX,
#endif
	ADC_COUNT,
}adc_e;

static SemaphoreHandle_t adcComplete [ADC_COUNT];

void adcInit(void){
	for (uint8_t i = 0; i < ADC_COUNT; ++i) {
		adcComplete[i] = xSemaphoreCreateBinary();
	}
}

int8_t adcIndex(ADC_HandleTypeDef* hadc){
	#ifdef ADC1_HANDLE
		if(hadc->Instance == ADC1) return ADC1_INDEX;
	#endif
	#ifdef ADC2_HANDLE
		if(hadc->Instance == ADC2) return ADC2_INDEX;
	#endif
	#ifdef ADC3_HANDLE
		if(hadc->Instance == ADC3) return ADC3_INDEX;
	#endif
	#ifdef ADC4_HANDLE
		if(hadc->Instance == ADC4) return ADC4_INDEX;
	#endif
	return -1;
}

int8_t adcRead(ADC_HandleTypeDef* hadc, uint32_t* pBuffer){
	int8_t ix = adcIndex(hadc);
	if(ix == -1) return HAL_ERROR;

	uint8_t status = HAL_ADC_Start_IT(hadc);
	if(status != HAL_OK) return status;

	if(xSemaphoreTake(adcComplete[ix], ADC_TIMEOUT) != pdTRUE) return HAL_TIMEOUT;

	*pBuffer = HAL_ADC_GetValue(hadc);
	return HAL_OK;
}

void HAL_ADC_ConvCpltCallback (ADC_HandleTypeDef *hadc){
	int8_t ix = adcIndex(hadc);
	if(ix == -1) return; /* WTF */

	HAL_ADC_Stop_IT(hadc);
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

	xSemaphoreGiveFromISR(adcComplete[ix],&xHigherPriorityTaskWoken);
	if(xHigherPriorityTaskWoken){
		portYIELD();
	}
}

#endif
