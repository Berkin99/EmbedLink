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

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <sysdefs.h>
#include "rtos.h"
#include "sysconfig.h"
#include "systime.h"
#include "uart.h"
#include "telemetry.h"
#include "northcom.h"
#include "ntrp.h"
#include "nrx_logic.h"
#include "nrx.h"
#include "led.h"
#include "rc_interface.h"
#include "uavcom.h"

#ifndef NC_ID
#define NC_ID         'X'
#endif

static int8_t              ncTRX;
static RC_Handle_t         ncRC;

static NTRP_Message_t      ncMessage;
static uint8_t             isInit = 0;
static uint32_t            lastDataTime = 0;

taskAllocateStatic(NCOM, NC_TASK_STACK, NC_TASK_PRI);
void ncTask(void* argv);

uint8_t ncInit(void){
    if(isInit) return E_OVERWRITE;
    nrxLogicInit();
    ncTRX = -1;

    ncRC = RC_NewHandle();
    uavcomInit();

    #ifdef NC_MODULE
    ncTRX = telemetryGetIndex(NC_MODULE);
    #endif

    if(ncTRX < 0){
        serialPrint("[-] NorthCom Init ERROR\n");
        return E_NOT_FOUND;
    }

    taskCreateStatic(NCOM, ncTask, NULL);
    serialPrint("[+] NorthCom Init OK\n");
    isInit = 1;
    return OK;
}

void ncTask(void* argv){
    static uint8_t rxBuffer[NTRP_MAX_MSG_SIZE];

    while(!telemetryIsReady(ncTRX)) delay(10);

    while(1)
    {
        telemetryWaitDataReady(ncTRX);
        if(telemetryReceive(ncTRX, rxBuffer) > 0){
			#ifdef NC_RX_LED
            ledToggle(NC_RX_LED);
			#endif
            ncDataHandler(rxBuffer);
        }
    }
}

uint32_t ncLastDataTime(void){
    return lastDataTime;
}

int8_t ncTransmitPacket(NTRP_Packet_t* packet, uint8_t size){
    if(!isInit) return E_NOT_FOUND;


    #ifdef NC_NTRPMESSAGE
    uint8_t txBuffer[NTRP_MAX_MSG_SIZE];
    if(size > NTRP_MAX_MSG_SIZE) size = NTRP_MAX_MSG_SIZE;
    NTRP_Message_t ntrp_tx;
    ntrp_tx.talkerID = nc_ID;
    ntrp_tx.receiverID = NTRP_MASTER_ID;
    ntrp_tx.packetsize = size;
    ntrp_tx.packet = packet;
    if(NTRP_Unite(txBuffer, &ntrp_tx)) telemetryTransmit(ncTRX, txBuffer, size + 5);

    #elif defined(NC_NTRPPACKET)
    uint8_t txBuffer[NTRP_MAX_MSG_SIZE];

    if(size > NTRP_MAX_PACKET_SIZE) size = NTRP_MAX_PACKET_SIZE;

    if(NTRP_PackUnite(txBuffer, size, packet)){
        ledToggle(NC_TX_LED);
        return telemetryTransmit(ncTRX, txBuffer, size);
    }

    #endif
    return E_CONF_FAIL;
}

void ncDataHandler(const uint8_t* rxBuffer){
    uint8_t status = 0;

    #ifdef NC_NTRPMESSAGE
    status = NTRP_Parse(&ncMessage, rxBuffer);
    if(ncMessage.receiverID != nc_ID)return;

    #elif defined(NC_NTRPPACKET)
    status = NTRP_PackParse(&ncMessage.packet, rxBuffer);
    #endif

    if(status != 1) return;

    lastDataTime = millis();
    ncPacketHandler(&ncMessage.packet);
}

void ncPacketHandler(NTRP_Packet_t* packet){
    switch (packet->header) {
        case NTRP_NAK:ncTransmitPacket(&ncMessage.packet, ncMessage.packetsize);break;
        case NTRP_ACK:RxACK();break;
        case NTRP_MSG:RxMSG(packet->data.bytes,packet->dataID);break;
        case NTRP_CMD:RxCMD(packet->dataID,packet->data.bytes);break;
        case NTRP_GET:RxGET(packet->dataID);break;
        case NTRP_SET:RxSET(packet->dataID,packet->data.bytes);break;
        default:break;
    }
}

void ncDebug(char* format, ...)
{
    va_list args;
    char temp[104] = {0};

    va_start(args, format);
    vsprintf(temp, format, args);
    va_end(args);

    temp[103] = 0x00;

    int i = 0;
    while(temp[i] != 0x00){
        if(i % (NTRP_MAX_PACKET_SIZE - 2) == 0){
            TxMSG(&temp[i]);
        }
        i++;
    }
}

void RxACK(void){
    /* Rx Ack Event */
    return;
}

void TxACK(void){
    NTRP_Packet_t packet;
    packet.header = NTRP_ACK;
    packet.dataID = 1;
    ncTransmitPacket(&packet, 2);
}

void RxMSG(const uint8_t* msg, uint8_t len){
    static char temp [27];
    uint8_t i;
    for (i = 0; i < len;i++){
        temp[i] = (char)msg[i];
        if(i >= 26) break;
    }
    temp[i] = 0x00;
    serialPrint("[>] NCOM Received: %s\n",temp);
    TxACK();
}

void TxMSG(const char* msg){
    NTRP_Packet_t packet;
    packet.header = NTRP_MSG;
    uint8_t i = 0;
    while(i < NTRP_MAX_PACKET_SIZE - 2){
        if(msg[i] == 0) break;
        packet.data.bytes[i] = msg[i];
        i++;
    }
    packet.dataID = i;
    ncTransmitPacket(&packet, i + 2);
}

void RxCMD(uint8_t cmdid, uint8_t* data){

    /* Command Modes */
#define RC_CONTROLLER          0x00
#define NRX_CONTENT_ID         0x01
#define FUNC_CONTENT_ID        0x02
#define UAV_CONTROLLER         40

    switch (cmdid){
    case (RC_CONTROLLER):{RC_Update(&ncRC, data);}break;
    case (UAV_CONTROLLER):{uavcomUpdate(data);}break;
    case (NRX_CONTENT_ID):{
        uint8_t arr[26] = {0};
        struct nrx_s* val = nrxGetVar(data[0]);
        if(val == NULL){TxACK();break;}
        arr[0] = data[0];
        arr[1] = val->type;
        strcpy((char*)&arr[2],val->name);
        //serialPrint("[>] NRX:%d:%d:%s\n",arr[0],arr[1],(char*)&arr[2]); /* Debug */
        TxCMD(NRX_CONTENT_ID, arr);
    }break;
    case (FUNC_CONTENT_ID):break;
    default:break;
    }
}

void TxCMD(uint8_t cmdid,const uint8_t* data){
    NTRP_Packet_t packet;
    packet.header = NTRP_CMD;
    packet.dataID = cmdid;

    for(uint8_t i = 0; i < 26; i++){
        packet.data.bytes[i] = data[i];
    }
    ncTransmitPacket(&packet,28);
}

void RxGET(uint8_t dataid){
    struct nrx_s* nrxptr = nrxGetVar(dataid);
    if(nrxptr==NULL){ serialPrint("[>] NCOM nrx not found: %d\n", dataid); return;}

    NTRP_Packet_t packet;
    packet.header = NTRP_SET;
    packet.dataID = dataid;

    uint8_t  byteindex = 0;
    uint8_t  isGroup = 0;
    uint8_t  datasize;
    uint8_t* ptr;

    if(nrxptr->type & (NRX_GROUP)){
        if(!(nrxptr->type & (NRX_START))) { serialPrint("[>] NCOM nrx is not start: %d\n", dataid); return;}
        isGroup = 1;
        nrxptr++;
    }

    do {
        if(nrxptr->type & (NRX_GROUP)) {
            isGroup = 0;
            break;
        }

        datasize = nrxVarSize(nrxptr->type);
        ptr = (uint8_t*)nrxptr->address;
        for(uint8_t j = 0; j < datasize; j++){
            packet.data.bytes[byteindex] = *ptr;
            ptr++;
            byteindex++;
        }
        nrxptr++;
    } while (byteindex < 26 && isGroup);

    ncTransmitPacket(&packet, (byteindex + 2));
}

void RxSET(uint8_t dataid, uint8_t* data)
{
    struct nrx_s* nrxptr = nrxGetVar(dataid);
    if(nrxptr==NULL) return;

    uint8_t byteindex = 0;
    uint8_t isGroup = 0;
    uint8_t datasize;
    uint8_t* ptr;

    if(nrxptr->type & (NRX_GROUP)){
        if(!(nrxptr->type & (NRX_START))) return;
        isGroup = 1;
        nrxptr++;
    }

    do {
        if(nrxptr->type & (NRX_GROUP)) {isGroup = 0; break;}

        datasize = nrxVarSize(nrxptr->type);
        ptr = (uint8_t*) nrxptr->address;
        for(uint8_t j = 0; j < datasize; j++){
            *ptr = data[byteindex];
            ptr++;
            byteindex++;
        }
        nrxptr++;
    } while (byteindex < 26 && isGroup);
}

void TxSET(uint8_t dataid, void* bytes, uint8_t size){
    NTRP_Packet_t packet;
    packet.header = NTRP_SET;
    packet.dataID = dataid;

    for(uint8_t i = 0; i<size; i++){
        packet.data.bytes[i] = ((uint8_t*)bytes)[i];
    }

    ncTransmitPacket(&packet, (size+2));
}
