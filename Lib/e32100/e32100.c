/*
 *  e32100.c
 *
 *  Created on: Dec 22, 2023
 *  Author: BerkN
 *
 *  E-Byte E32-TTL-100 Transceiver Module Driver
 *  E32-TTL-100 Module is based on Semtech SX1278 chip.
 *  UART communication.
 *  Changeable all possible settings.
 *
 *  Updates and bug reports :  @ https://github.com/Berkin99/E32-TTL-100
 *
 *  22.12.2023 : Created E32-TTL-100 module driver.
 *  12.01.2024 : AUX Pin taken into account.
 *  14.01.2024 : UART Receive DMA abort (bugfix).
 *  22.01.2024 : DMA is optional.
 *  02.09.2024 : Object System
 *
 *  References:
 *  [0] e32-ttl-100-datasheet-en-v1-0.pdf
 *  [1] semtech-sx127x-series-datasheet.pdf
 *
 */

#include <stdint.h>
#include "e32100.h"

E32100_Device_t E32100_NewDevice(void* pIntf, void* pinM0, void* pinM1, void* pinAUX,
    E32100_PinRead_t getf, E32100_PinWrite_t setf, E32100_Read_t readf, E32100_Write_t writef, E32100_Delay_t delayf){

    E32100_Device_t new = {
        .mode = E32100_MODE_NORMAL,
        .pIntf = pIntf,
        .M0 = pinM0,
        .M1 = pinM1,
        .AUX = pinAUX,
        .pinRead = getf,
		.pinWrite = setf,
        .read = readf,
        .write = writef,
        .delay = delayf
    };

    return new;
}

void E32100_Init(E32100_Device_t* self){
    E32100_SetMode(self, E32100_MODE_NORMAL);
    E32100_WaitAUX(self, 1000);
}

int8_t E32100_TestConnection(E32100_Device_t* self){
    if(!self->pinRead(self->AUX) ){
        E32100_WaitAUX(self, 1000);
        if(!self->pinRead(self->AUX) ) return E32100_ERROR;
    }
    return E32100_OK;
}

void E32100_SetMode(E32100_Device_t* self, E32100_Mode_e mode){

    if(!self->pinRead(self->AUX)){
        E32100_WaitAUX(self, 1000);
        if(!self->pinRead(self->AUX)) return;
        self->delay(E32100_AUX_CHANGE_INTERVAL);
    }

    uint8_t _m0 = 0;
    uint8_t _m1 = 0;

    if(((uint8_t)mode & (1U<<0)) > 0) _m0 = 1;
    if(((uint8_t)mode & (1U<<1)) > 0) _m1 = 1;

    self->pinWrite(self->M0, _m0);
    self->pinWrite(self->M1, _m1);
    self->mode = mode;

    self->delay(E32100_MODE_CHANGE_INTERVAL);
}

void E32100_SetConfig(E32100_Device_t* self, E32100_Config_t config, uint8_t save){

    E32100_SetMode(self, E32100_MODE_SLEEP);

    uint8_t param[6];

    if(save) param[0] = E32100_CMD_SAVE_ON;
    else param[0] = E32100_CMD_SAVE_OFF;

    param[1] = config.addh;
    param[2] = config.addl;
    param[3] = E32100_SpedByte(config.sped);
    param[4] = E32100_ChannelByte(config.channel);
    param[5] = E32100_OptionByte(config.option);

    self->write(self->pIntf, param, 6);

    self->delay(E32100_COMMAND_INTERVAL);
}

E32100_Config_t E32100_GetDefaultConfig(void){
    static E32100_Config_t config = {
        .addh = E32100_DEFAULT_ADDH,
        .addl = E32100_DEFAULT_ADDL,
        .sped = {E32100_ADR_2400, E32100_BAUDRATE_9600, E32100_PARITY_8N1}, /* Sped */
        .channel = E32100_DEFAULT_CHANNEL,
        .option = {E32100_TXPOWER_20, E32100_FEC_OFF, E32100_WAKEUP_250, E32100_IOMODE_ON, E32100_TXMODE_TRANSPARENT} /* Option */
    };
    return config;
}

void E32100_SetDefaultConfig(E32100_Device_t* self, uint8_t save){
    E32100_SetConfig(self, E32100_GetDefaultConfig(), save);
}

void E32100_Command(E32100_Device_t* self, E32100_Command_e cmd){
    if(self->mode != E32100_MODE_SLEEP) E32100_SetMode(self, E32100_MODE_SLEEP);

    if(!self->pinRead(self->AUX)){
        E32100_WaitAUX(self, 100);
        if(!self->pinRead(self->AUX)) return;
        self->delay(E32100_AUX_CHANGE_INTERVAL);
    }

    uint8_t param[3] = {cmd, cmd, cmd};
    self->write(self->pIntf, param, 3);
}

void E32100_Reset(E32100_Device_t* self){
    E32100_Command(self, E32100_CMD_RESET);
    E32100_SetMode(self, E32100_MODE_NORMAL);
    E32100_WaitAUX(self, 1000);
}

void E32100_GetConfig(E32100_Device_t* self, uint8_t* buffer){
    E32100_Command(self, E32100_CMD_READ_CFG);
    self->read(self->pIntf, buffer, 6);
}

void E32100_GetModuleVersion(E32100_Device_t* self, uint8_t* buffer){
    E32100_Command(self, E32100_CMD_MODULE);
    self->read(self->pIntf, buffer, 5);
}

void E32100_WaitAUX(E32100_Device_t* self, uint16_t timeout){
    uint16_t cnt = 0;
    while(timeout > cnt && !self->pinRead(self->AUX)){
        self->delay(1);
        cnt++;
    }
}

int8_t E32100_Write(E32100_Device_t* self, const uint8_t* pTxData, uint16_t size){
    if(!self->pinRead(self->AUX)){
        E32100_WaitAUX(self, E32100_TIMEOUT);
        if(!self->pinRead(self->AUX)) return E32100_ERROR;
        self->delay(E32100_AUX_CHANGE_INTERVAL);
    }
    return self->write(self->pIntf, pTxData, size);
}

int8_t E32100_Read(E32100_Device_t* self, uint8_t *pRxData, uint16_t size){
    return self->read(self->pIntf, pRxData, size);
}

uint8_t E32100_SpedByte (E32100_Sped_t sped){
    uint8_t temp = ((uint8_t)(sped.airDataRate) << E32100_OFFSET_AIRDATARATE);
    temp |= ((uint8_t)(sped.baudRate) << E32100_OFFSET_BAUDRATE);
    temp |= ((uint8_t)(sped.parity) << E32100_OFFSET_PARITY);
    return temp;
}

uint8_t E32100_OptionByte (E32100_Option_t option){
    uint8_t temp = ((uint8_t)(option.txPower) << E32100_OFFSET_TXPOWER);
    temp |= ((uint8_t)(option.fec) << E32100_OFFSET_FEC);
    temp |= ((uint8_t)(option.wakeUpTime) << E32100_OFFSET_WAKEUP);
    temp |= ((uint8_t)(option.ioMode) << E32100_OFFSET_IOMODE);
    temp |= ((uint8_t)(option.txMode) << E32100_OFFSET_TXMODE);
    return temp;
}

uint8_t E32100_ChannelByte (uint16_t channel){
    return (uint8_t)(channel - 410);
}
