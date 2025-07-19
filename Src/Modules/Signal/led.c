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

#include "led.h"
#include "gpio.h"

typedef struct {
	uint8_t value;
	uint16_t pin;
}led_t;

static led_t led[LED_COUNT] = {
	#ifdef LED1_PIN
			{.value = 0, .pin = (LED1_PIN)},
	#endif
	#ifdef LED2_PIN
			{.value = 0, .pin = (LED2_PIN)},
	#endif
	#ifdef LED3_PIN
			{.value = 0, .pin = (LED3_PIN)},
	#endif
	#ifdef LED4_PIN
			{.value = 0, .pin = (LED4_PIN)},
	#endif
	#ifdef LED5_PIN
			{.value = 0, .pin = (LED5_PIN)},
	#endif
	#ifdef LED6_PIN
			{.value = 0, .pin = (LED6_PIN)},
	#endif
	#ifdef LED7_PIN
			{.value = 0, .pin = (LED7_PIN)},
	#endif
	#ifdef LED8_PIN
			{.value = 0, .pin = (LED8_PIN)},
	#endif
};

void ledSet(uint8_t index, uint8_t value){
	led[index].value = value;
	pinWrite(led[index].pin, value);
}

uint8_t ledGet(uint8_t index){
	return led[index].value;
}

void ledToggle(uint8_t index){
	ledSet(index, !(ledGet(index)));
}
