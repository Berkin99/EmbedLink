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

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "system.h"
#include "uart.h"
#include "rtos.h"

#define UART_TIMEOUT (1000)

uart_t uart1;
uart_t uart2;
uart_t uart3;
uart_t uart4;

void uartInit(void){
#ifdef HUART1
    uart1.handle = &HUART1;
    uart1.mutex  = mutexCreate();
    uart1.txCplt = semaphoreCreate();
    uart1.rxCplt = semaphoreCreate();
#endif
#ifdef HUART2
    uart2.handle = &HUART2;
    uart2.mutex  = mutexCreate();
    uart2.txCplt = semaphoreCreate();
    uart2.rxCplt = semaphoreCreate();
#endif
#ifdef HUART3
    uart3.handle = &HUART3;
    uart3.mutex  = mutexCreate();
    uart3.txCplt = semaphoreCreate();
    uart3.rxCplt = semaphoreCreate();
#endif
#ifdef HUART4
    uart4.handle = &HUART4;
    uart4.mutex  = mutexCreate();
    uart4.txCplt = semaphoreCreate();
    uart4.rxCplt = semaphoreCreate();
#endif
}

void uartSetBaudRate(uart_t* uart, uint32_t rate){
    if(mutexTake(uart->mutex, RTOS_MAX_DELAY) != RTOS_OK) return;
    HAL_UART_DeInit(uart->handle);
    ((UART_HandleTypeDef*)uart->handle)->Init.BaudRate = rate;
    HAL_UART_Init(uart->handle);
    mutexGive(uart->mutex);
}

uint32_t uartGetBaudRate(uart_t* uart){
    return ((UART_HandleTypeDef*)uart->handle)->Init.BaudRate;
}

int8_t uartRead(uart_t* uart, uint8_t* pRxData, uint16_t len){
    if(mutexTake(uart->mutex, UART_TIMEOUT) != RTOS_OK) return HAL_ERROR;
    HAL_UART_Receive_IT(uart->handle, pRxData, len);
    if(semaphoreTake(uart->rxCplt, UART_TIMEOUT) != RTOS_OK){
        mutexGive(uart->mutex);
        return HAL_ERROR;
    }
    mutexGive(uart->mutex);
    return HAL_OK;
}

int8_t uartReadToIdle (uart_t* uart, uint8_t* pRxData, uint16_t len){
    if(mutexTake(uart->mutex, UART_TIMEOUT) != RTOS_OK) return HAL_ERROR;
    HAL_UARTEx_ReceiveToIdle_IT(uart->handle, pRxData, len);
    if(semaphoreTake(uart->rxCplt, UART_TIMEOUT) != RTOS_OK){
        mutexGive(uart->mutex);
        return HAL_ERROR;
    }
    mutexGive(uart->mutex);
    return HAL_OK;
}

int8_t uartWrite (uart_t* uart, const uint8_t* pTxData, uint16_t len){
    if(mutexTake(uart->mutex, UART_TIMEOUT) != RTOS_OK) return HAL_ERROR;
    HAL_UART_Transmit_IT(uart->handle, pTxData, len);
    if(semaphoreTake(uart->txCplt, UART_TIMEOUT) != RTOS_OK){
        mutexGive(uart->mutex);
        return HAL_ERROR;
    }
    mutexGive(uart->mutex);
    return HAL_OK;
}

int8_t uartPrint(uart_t* uart, const char* format, ...) {
    va_list args;
    char buffer[128];
    int len;

    va_start(args, format);
    len = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

   return uartWrite(uart, (uint8_t*)buffer, len);
}

#ifdef SERIAL_UART
void serialPrint (char* format, ...){
	va_list args;
	char buffer[128];
	int len;

	va_start(args, format);
	length = vsnprintf(buffer, sizeof(buffer), format, args);
	va_end(args);

	uartWrite(SERIAL_UART, buffer, len);
}
#endif

uart_t* HAL_UART_Parent(UART_HandleTypeDef* huart){
    #ifdef HUART1
    if(huart == &HUART1) return &uart1;
    #endif
    #ifdef HUART2
    if(huart == &HUART2) return &uart2;
    #endif
    #ifdef HUART3
    if(huart == &HUART3) return &uart3;
    #endif
    #ifdef HUART4
    if(huart == &HUART4) return &uart4;
    #endif
    return NULL;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
    uart_t* parent = HAL_UART_Parent(huart);
    if(parent == NULL) return;
    semaphoreGiveISR(parent->rxCplt);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
    uart_t* parent = HAL_UART_Parent(huart);
    if(parent == NULL) return;
    semaphoreGiveISR(parent->txCplt);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){
    uart_t* parent = HAL_UART_Parent(huart);
    if(parent == NULL) return;
    parent->received = Size;
    semaphoreGiveISR(parent->rxCplt);
}
