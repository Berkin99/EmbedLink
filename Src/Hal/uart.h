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

#ifndef UART_H_
#define UART_H_

#include <stdarg.h>
#include <stdint.h>
#include "sysconfig.h"

typedef struct uart_s uart_t;

extern uart_t uart1;
extern uart_t uart2;
extern uart_t uart3;
extern uart_t uart4;

void     uartInit         (void);
int8_t   uartBegin        (uart_t* uart);
void     uartSetBaudRate  (uart_t* uart, uint32_t rate);
uint32_t uartGetBaudRate  (uart_t* uart);
uint8_t  uartAvailable    (uart_t* uart);
uint8_t  uartRead         (uart_t* uart);
uint8_t  uartPeek         (uart_t* uart);
int8_t   uartWrite        (uart_t* uart, const uint8_t *data, uint16_t len);
int8_t   uartPrint        (uart_t* uart, const char* format, ...);
int8_t   uartReceive      (uart_t* uart, uint8_t* pRxData, uint16_t len);
int8_t   uartReceiveToIdle(uart_t* uart, uint8_t* pRxData, uint16_t len);
int8_t   uartTransmit     (uart_t* uart, const uint8_t* pTxData, uint16_t len);

#ifdef SERIAL_UART

void     serialPrint     (const char* format, ...);
int32_t  serialScan      (const char *format, ...);
uint8_t  serialAvailable (void);
uint8_t  serialRead      (void);
uint8_t  serialPeek      (void);
int8_t   serialWrite     (const uint8_t *data, uint16_t len);

#endif

#endif /* UART_H_ */