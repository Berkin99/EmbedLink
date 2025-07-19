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

#ifndef LED_H_
#define LED_H_

#include <stdint.h>
#include "sysconfig.h"

#define LED_LOW     0
#define LED_HIGH    1

typedef enum{
	#ifdef LED1_PIN
		LED1,
	#endif
	#ifdef LED2_PIN
		LED2,
	#endif
	#ifdef LED3_PIN
		LED3,
	#endif
	#ifdef LED4_PIN
		LED4,
	#endif
	#ifdef LED5_PIN
		LED5,
	#endif
	#ifdef LED6_PIN
		LED6,
	#endif
	#ifdef LED7_PIN
		LED7,
	#endif
	#ifdef LED8_PIN
		LED8,
	#endif
	LED_COUNT
}led_e;

void ledSet    (led_e index, uint8_t value);
uint8_t ledGet (led_e index);
void ledToggle (led_e index);

#endif /* LED_H_ */
