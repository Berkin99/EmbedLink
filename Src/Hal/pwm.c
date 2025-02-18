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
#include "pwm.h"

#ifdef HAL_TIM_MODULE_ENABLED

#define PERIPH_FREQ			200000000	/* AHB1 AHB2 Clock Frequency in Hz */
#define PERIPH_PERIOD_MS	1000.0f/(float)PERIPH_FREQ
#define PERIPH_PERIOD_US 	1000000.0f/(float)PERIPH_FREQ
#define TIMER_CH(TIMER, CH) ((&(TIMER.Instance->CCR1)) - 1 + CH)

pwm_t pwm1;
pwm_t pwm2;
pwm_t pwm3;
pwm_t pwm4;

void pwmInit(void){

	#ifdef PWM1_TIMER
	pwm1.handle = &PWM1_TIMER;
	pwm1.ch = PWM1_CH;
	pwm1.reg = TIMER_CH(PWM1_TIMER, PWM1_CH);
	pwm1.frequency = PERIPH_FREQ / ((PWM1_TIMER.Init.Period + 1) * (PWM1_TIMER.Init.Prescaler + 1));
	pwm1.usPerReg = (float)(PWM1_TIMER.Init.Prescaler + 1) * PERIPH_PERIOD_US;
	#endif
	#ifdef PWM2_TIMER
	pwm2.handle = &PWM2_TIMER;
	pwm2.ch = PWM2_CH;
	pwm2.reg = TIMER_CH(PWM2_TIMER, PWM2_CH);
	pwm2.frequency = PERIPH_FREQ / ((PWM2_TIMER.Init.Period + 1) * (PWM2_TIMER.Init.Prescaler + 1));
	pwm2.usPerReg = (float)(PWM2_TIMER.Init.Prescaler + 1) * PERIPH_PERIOD_US;
	#endif
	#ifdef PWM3_TIMER
	pwm3.handle = &PWM3_TIMER;
	pwm3.ch = PWM3_CH;
	pwm3.reg = TIMER_CH(PWM3_TIMER, PWM3_CH);
	pwm3.frequency = PERIPH_FREQ / ((PWM3_TIMER.Init.Period + 1) * (PWM3_TIMER.Init.Prescaler + 1));
	pwm3.usPerReg = (float)(PWM3_TIMER.Init.Prescaler + 1) * PERIPH_PERIOD_US;
	#endif
	#ifdef PWM4_TIMER
	pwm4.handle = &PWM4_TIMER;
	pwm4.ch = PWM4_CH;
	pwm4.reg = TIMER_CH(PWM4_TIMER, PWM4_CH);
	pwm4.frequency = PERIPH_FREQ / ((PWM4_TIMER.Init.Period + 1) * (PWM4_TIMER.Init.Prescaler + 1));
	pwm4.usPerReg = (float)(PWM4_TIMER.Init.Prescaler + 1) * PERIPH_PERIOD_US;
	#endif
}

void pwmStart (pwm_t* pwm){
	HAL_TIM_PWM_Start(pwm->handle, (pwm->ch - 1) * 4 );
}

void pwmStop (pwm_t* pwm){
	HAL_TIM_PWM_Stop(pwm->handle, (pwm->ch - 1) * 4 );
}

void pwmWrite (pwm_t* pwm, float us){
	/* us/(us/ccr) = us * (ccr/us) -> us */
	*pwm->reg = (uint32_t)((us) / pwm->usPerReg);
}

float pwmRead (pwm_t* pwm){
	return (float) *(pwm->reg) * (pwm->usPerReg);
}

#endif
