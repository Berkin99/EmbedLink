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

#ifdef HAL_PCD_MODULE_ENABLED

#define USB_TIMEOUT (1000)

extern USBD_HandleTypeDef hUsbDeviceFS;

void usbInit(void)
{
}

int8_t usbReceive(uint8_t* pRxData, uint16_t len)
{
    if (CDC_Receive_FS(pRxData, &len) != USBD_OK)
        return E_CONNECTION;
    return OK;
}

int8_t usbTransmit(uint8_t* pTxData, uint16_t len)
{
    if (CDC_Transmit_FS(pTxData, len) != USBD_OK)
        return E_CONNECTION;
    return OK;
}

uint16_t usbAvailableData(void)
{
    extern uint16_t UserRxLengthFS;
    return UserRxLengthFS;
}

void usbWaitDataReady(void)
{
    while (usbAvailableData() == 0);
}

void usbPrint(char* format, ...)
{
    char buffer[128];
    va_list args;
    va_start(args, format);
    int len = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    usbTransmit((uint8_t*)buffer, len);
}

#endif
