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

#include "uart.h"
#include "system.h"
#include "sysconfig.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

#ifdef HAL_UART_MODULE_ENABLED

#define UART_TIMEOUT (1000)

struct uart_s {
    UART_HandleTypeDef *handle;
};

uart_t uart1;
uart_t uart2;
uart_t uart3;
uart_t uart4;

void uartInit(void)
{
#ifdef HUART1
    uart1.handle = &HUART1;
#endif
#ifdef HUART2
    uart2.handle = &HUART2;
#endif
#ifdef HUART3
    uart3.handle = &HUART3;
#endif
#ifdef HUART4
    uart4.handle = &HUART4;
#endif
}

void uartSetBaudRate(uart_t* uart, uint32_t rate){
    HAL_UART_DeInit(uart->handle);
    uart->handle->Init.BaudRate = rate;
    HAL_UART_Init(uart->handle);
}

uint32_t uartGetBaudRate(uart_t* uart){
    return uart->handle->Init.BaudRate;
}

int8_t uartRead(uart_t* uart, uint8_t* pRxData, uint16_t len){
    if (HAL_UART_Receive(uart->handle, pRxData, len, UART_TIMEOUT) != HAL_OK) return E_CONNECTION;
    return OK;
}

int8_t uartReadToIdle(uart_t* uart, uint8_t* pRxData, uint16_t len){
	uint16_t temp;
    if (HAL_UARTEx_ReceiveToIdle(uart->handle, pRxData, len, &temp, UART_TIMEOUT) != HAL_OK) return E_CONNECTION;
    return OK;
}

int8_t uartWrite(uart_t* uart, const uint8_t* pTxData, uint16_t len){
    if (HAL_UART_Transmit(uart->handle, (uint8_t*)pTxData, len, UART_TIMEOUT) != HAL_OK) return E_CONNECTION;
    return OK;
}

int8_t uartPrint(uart_t* uart, const char* format, ...){
    char buffer[128];
    va_list args;
    va_start(args, format);
    int len = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    return uartWrite(uart, (uint8_t*)buffer, len);
}

#ifdef SERIAL_UART

void serialPrint(const char* format, ...){
    char buffer[128];
    va_list args;
    va_start(args, format);
    uint16_t len = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    uartWrite(&SERIAL_UART, (uint8_t*)buffer, len);
}

int32_t serialScan(const char *format, ...){
    char buffer[128];
    uint16_t idx = 0;
    uint8_t ch;
    int32_t result = 0;

    while (idx < sizeof(buffer) - 1) {
        if (HAL_UART_Receive(SERIAL_UART.handle, &ch, 1, HAL_MAX_DELAY) != HAL_OK) continue;
        if (ch == '\r') continue;
        if (ch == '\n') break;
        buffer[idx++] = ch;
    }
    buffer[idx] = '\0';
    serialPrint("%s\n", buffer);
    if (idx == 0) return 0;

    va_list args;
    va_start(args, format);

    const char *f = format;
    char *s = buffer;
    while (*f) {
        while (isspace((unsigned char)*f)) ++f;
        while (isspace((unsigned char)*s)) ++s;
        if (*f == '%') {
            ++f;
            if (*f == 'd') {
                int *iptr = va_arg(args, int *);
                *iptr = strtol(s, &s, 10);
                result++;
            } else if (*f == 'f') {
                float *fptr = va_arg(args, float *);
                *fptr = strtof(s, &s);
                result++;
            } else if (*f == 'u') {
                unsigned *uptr = va_arg(args, unsigned *);
                *uptr = strtoul(s, &s, 10);
                result++;
            } else if (*f == 's') {
                char *sptr = va_arg(args, char *);
                sscanf(s, "%s", sptr);
                result++;
            } else if (*f == 'c') {
                char *cptr = va_arg(args, char *);
                *cptr = *s++;
                result++;
            }
        }
        ++f;
    }

    va_end(args);
    return result;
}
#endif

#endif
