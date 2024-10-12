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

#ifndef GPIO_H_
#define GPIO_H_

#include <stdint.h>

#define PORT_A     (0x00 << 8)
#define PORT_B     (0x01 << 8)
#define PORT_C     (0x02 << 8)
#define PORT_D     (0x03 << 8)
#define PORT_E     (0x04 << 8)
#define PORT_F     (0x05 << 8)

#define PA0        (uint16_t)(PORT_A | 0)
#define PA1        (uint16_t)(PORT_A | 1)
#define PA2        (uint16_t)(PORT_A | 2)
#define PA3        (uint16_t)(PORT_A | 3)
#define PA4        (uint16_t)(PORT_A | 4)
#define PA5        (uint16_t)(PORT_A | 5)
#define PA6        (uint16_t)(PORT_A | 6)
#define PA7        (uint16_t)(PORT_A | 7)
#define PA8        (uint16_t)(PORT_A | 8)
#define PA9        (uint16_t)(PORT_A | 9)
#define PA10       (uint16_t)(PORT_A | 10)
#define PA11       (uint16_t)(PORT_A | 11)
#define PA12       (uint16_t)(PORT_A | 12)
#define PA13       (uint16_t)(PORT_A | 13)
#define PA14       (uint16_t)(PORT_A | 14)
#define PA15       (uint16_t)(PORT_A | 15)

#define PB0        (uint16_t)(PORT_B | 0)
#define PB1        (uint16_t)(PORT_B | 1)
#define PB2        (uint16_t)(PORT_B | 2)
#define PB3        (uint16_t)(PORT_B | 3)
#define PB4        (uint16_t)(PORT_B | 4)
#define PB5        (uint16_t)(PORT_B | 5)
#define PB6        (uint16_t)(PORT_B | 6)
#define PB7        (uint16_t)(PORT_B | 7)
#define PB8        (uint16_t)(PORT_B | 8)
#define PB9        (uint16_t)(PORT_B | 9)
#define PB10       (uint16_t)(PORT_B | 10)
#define PB11       (uint16_t)(PORT_B | 11)
#define PB12       (uint16_t)(PORT_B | 12)
#define PB13       (uint16_t)(PORT_B | 13)
#define PB14       (uint16_t)(PORT_B | 14)
#define PB15       (uint16_t)(PORT_B | 15)

#define PC0        (uint16_t)(PORT_C | 0)
#define PC1        (uint16_t)(PORT_C | 1)
#define PC2        (uint16_t)(PORT_C | 2)
#define PC3        (uint16_t)(PORT_C | 3)
#define PC4        (uint16_t)(PORT_C | 4)
#define PC5        (uint16_t)(PORT_C | 5)
#define PC6        (uint16_t)(PORT_C | 6)
#define PC7        (uint16_t)(PORT_C | 7)
#define PC8        (uint16_t)(PORT_C | 8)
#define PC9        (uint16_t)(PORT_C | 9)
#define PC10       (uint16_t)(PORT_C | 10)
#define PC11       (uint16_t)(PORT_C | 11)
#define PC12       (uint16_t)(PORT_C | 12)
#define PC13       (uint16_t)(PORT_C | 13)
#define PC14       (uint16_t)(PORT_C | 14)
#define PC15       (uint16_t)(PORT_C | 15)

#define PD0        (uint16_t)(PORT_D | 0)
#define PD1        (uint16_t)(PORT_D | 1)
#define PD2        (uint16_t)(PORT_D | 2)
#define PD3        (uint16_t)(PORT_D | 3)
#define PD4        (uint16_t)(PORT_D | 4)
#define PD5        (uint16_t)(PORT_D | 5)
#define PD6        (uint16_t)(PORT_D | 6)
#define PD7        (uint16_t)(PORT_D | 7)
#define PD8        (uint16_t)(PORT_D | 8)
#define PD9        (uint16_t)(PORT_D | 9)
#define PD10       (uint16_t)(PORT_D | 10)
#define PD11       (uint16_t)(PORT_D | 11)
#define PD12       (uint16_t)(PORT_D | 12)
#define PD13       (uint16_t)(PORT_D | 13)
#define PD14       (uint16_t)(PORT_D | 14)
#define PD15       (uint16_t)(PORT_D | 15)

#define PE0        (uint16_t)(PORT_E | 0)
#define PE1        (uint16_t)(PORT_E | 1)
#define PE2        (uint16_t)(PORT_E | 2)
#define PE3        (uint16_t)(PORT_E | 3)
#define PE4        (uint16_t)(PORT_E | 4)
#define PE5        (uint16_t)(PORT_E | 5)
#define PE6        (uint16_t)(PORT_E | 6)
#define PE7        (uint16_t)(PORT_E | 7)
#define PE8        (uint16_t)(PORT_E | 8)
#define PE9        (uint16_t)(PORT_E | 9)
#define PE10       (uint16_t)(PORT_E | 10)
#define PE11       (uint16_t)(PORT_E | 11)
#define PE12       (uint16_t)(PORT_E | 12)
#define PE13       (uint16_t)(PORT_E | 13)
#define PE14       (uint16_t)(PORT_E | 14)
#define PE15       (uint16_t)(PORT_E | 15)

#define PF0        (uint16_t)(PORT_F | 0)
#define PF1        (uint16_t)(PORT_F | 1)
#define PF2        (uint16_t)(PORT_F | 2)
#define PF3        (uint16_t)(PORT_F | 3)
#define PF4        (uint16_t)(PORT_F | 4)
#define PF5        (uint16_t)(PORT_F | 5)
#define PF6        (uint16_t)(PORT_F | 6)
#define PF7        (uint16_t)(PORT_F | 7)
#define PF8        (uint16_t)(PORT_F | 8)
#define PF9        (uint16_t)(PORT_F | 9)
#define PF10       (uint16_t)(PORT_F | 10)
#define PF11       (uint16_t)(PORT_F | 11)
#define PF12       (uint16_t)(PORT_F | 12)
#define PF13       (uint16_t)(PORT_F | 13)
#define PF14       (uint16_t)(PORT_F | 14)
#define PF15       (uint16_t)(PORT_F | 15)

typedef uint16_t pin_t;

void   pinMode  (pin_t pin, uint8_t mode);
void   pinWrite (pin_t pin, uint8_t state);
void   pinToggle(pin_t pin);
int8_t pinRead  (pin_t pin);

#endif /* GPIO_H_ */
