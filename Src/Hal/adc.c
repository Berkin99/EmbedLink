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

#include "adc.h"

#include "sysconfig.h"
#include "system.h"

#ifdef HAL_ADC_MODULE_ENABLED

#define ADC_TIMEOUT (1000)

struct adc_s {
    ADC_HandleTypeDef *handle;
};

adc_t adc1;
adc_t adc2;

void adcInit(void){
#ifdef HADC1
    adc1.handle = &HADC1;
#endif
#ifdef HADC2
    adc2.handle = &HADC2;
#endif
}

int8_t adcRead(adc_t* adc, uint32_t* pBuffer){
    if (HAL_ADC_Start(adc->handle) != HAL_OK) return E_ERROR;

    if (HAL_ADC_PollForConversion(adc->handle, ADC_TIMEOUT) != HAL_OK) {
        HAL_ADC_Stop(adc->handle);
        return E_TIMEOUT;
    }

    *pBuffer = HAL_ADC_GetValue(adc->handle);
    HAL_ADC_Stop(adc->handle);

    return OK;
}

#endif
