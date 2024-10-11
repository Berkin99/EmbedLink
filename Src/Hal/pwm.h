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

#ifndef PWM_H_
#define PWM_H_

#include <stdint.h>

typedef struct pwm_s{
	void* handle;
	uint8_t  ch;
	volatile uint32_t* reg;
	uint32_t  frequency;
	float 	  usPerReg;
}pwm_t;

extern pwm_t pwm1;
extern pwm_t pwm2;
extern pwm_t pwm3;
extern pwm_t pwm4;

void    pwmInit  (void);
void    pwmStart (pwm_t* pwm);
void    pwmStop  (pwm_t* pwm);
void    pwmWrite (pwm_t* pwm, float us);
float   pwmRead  (pwm_t* pwm);

#endif /* PWM_H_ */
