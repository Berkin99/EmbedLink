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

#include "usb.h"
#include "system.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

#ifdef __USBD_CDC_IF_H__

#define USB_TIMEOUT (1000)

static uint8_t* pBuffer;
static uint16_t length;

void usbInit(void){
    pBuffer = NULL;
    length = 0;
}

int8_t usbReceive(uint8_t* pRxData, uint16_t len){
    while (length == 0);
    if (len > length) len = length;
    memcpy(pRxData, pBuffer, len);
    length = 0;
    return OK;
}

int8_t usbTransmit(uint8_t* pTxData, uint16_t len){
    if (CDC_Transmit_FS(pTxData, len) != USBD_OK)
        return E_CONNECTION;
    uint32_t tick = HAL_GetTick();
    while (CDC_Transmit_FS(NULL, 0) == USBD_BUSY) {
        if (HAL_GetTick() - tick > USB_TIMEOUT)
            return E_TIMEOUT;
    }
    return OK;
}

uint16_t usbAvailableData(void){
    return length;
}

void usbWaitDataReady(void){
    while (length == 0);
}

void usbPrint(char* format, ...){
    char temp[128];
    va_list args;
    va_start(args, format);
    int len = vsnprintf(temp, sizeof(temp), format, args);
    va_end(args);
    usbTransmit((uint8_t*)temp, len);
}

void CDC_ReceiveCpltCallback(uint8_t* Buf, uint32_t* Len){
    pBuffer = Buf;
    length = (uint16_t)(*Len);
}

#endif
