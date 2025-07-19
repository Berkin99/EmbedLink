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

#ifndef NORTHCOM_H_
#define NORTHCOM_H_

#include <stdlib.h>
#include <stdint.h>
#include "ntrp.h"
#include "control.h"

#define NC_TASK_FREQ      250 //Hz

uint8_t  ncInit(void);
void 	 ncDataHandler    (const uint8_t* buffer);
void 	 ncPacketHandler  (NTRP_Packet_t* packet);
uint32_t ncLastDataTime   (void);
int8_t   ncTransmitPacket (NTRP_Packet_t* packet, uint8_t size);
void     ncDebug          (char* format, ...);

//void RxNAK(void);
//void TxNAK(void);
void RxACK(void);
void TxACK(void);
void RxMSG(const uint8_t* msg, uint8_t len);
void TxMSG(const char* msg);
void RxCMD(uint8_t cmdid, uint8_t* data);
void TxCMD(uint8_t cmdid, const uint8_t* data);
void RxGET(uint8_t dataid);
//void TxGET(void);	/*Agent Can't control Master Computer but want data from other agents (TOC need to be same)*/
void RxSET(uint8_t dataid, uint8_t* data);
void TxSET(uint8_t dataid, void* bytes, uint8_t size);

#endif /* NORTHCOM_H_ */
