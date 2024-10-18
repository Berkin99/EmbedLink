/*
 *  rf24.c
 *
 *  Created on: Fab 10, 2024
 *  Author: BerkN
 *
 *  NordicSemiconductor rf module driver.
 *  Platform independent C API.
 *  Updates and bug reports :  @ https://github.com/Berkin99/RF24
 *
 *  10.02.2024 : Created.
 *  19.03.2024 : Write Function added.
 *  14.10.2024 : Object System.
 *
 *  References:
 *  [0] nRF24L01_Product_Specification_v2_0-9199.pdf (Datasheet)
 *  [1] nRF24L01+ Single Chip 2.4GHz Transceiver Preliminary Product Specification v1.0
 *  [2] github.com/nRF24/RF24 (Register Map)
 * 
 */

#include <string.h>
#include "rf24.h"
#include "rf24_defs.h"
#include "rf24_user_defs.h"

#define RF24_MAX(a, b) (a > b ? a : b)
#define RF24_MIN(a, b) (a < b ? a : b)
#define _BV(x) (1 << (x))

#define RF24_POWERUP_DELAY   5
#define RF24_TXDELAY_250KBPS 155
#define RF24_TXDELAY_1MBPS   85
#define RF24_TXDELAY_2MBPS   65

static const uint8_t NOP                 = RF24_NOP;
static const uint8_t child_pipe[]        = {RX_ADDR_P0, RX_ADDR_P1, RX_ADDR_P2, RX_ADDR_P3, RX_ADDR_P4, RX_ADDR_P5};
static const uint8_t child_pipe_enable[] = {ERX_P0, ERX_P1, ERX_P2, ERX_P3, ERX_P4, ERX_P5};

RF24_Handle_t RF24_Init(void* intf, uint16_t ce, uint16_t cs){    
    RF24_Handle_t dev = {
        .payloadSize = 32,
        .configReg = 0,
        .txDelay = 85,
        ._isPlus = 0,
        ._isPipe0Rx = 0,
        ._isDynamic = 0,
        .intf = intf,
        .ce = ce,
        .cs = cs,
    };

    pinWrite(cs, RF24_PIN_HIGH); 

    return dev;
}

void beginTransaction(RF24_Handle_t* dev){
    spiBeginTransaction(dev->intf);  /* Mutex */
    pinWrite(dev->cs, RF24_PIN_LOW); /* Chip Select */
    delay(RF24_CS_DELAY_MS);
}

void endTransaction(RF24_Handle_t* dev){
    pinWrite(dev->cs, RF24_PIN_HIGH);
    spiEndTransaction(dev->intf);
}

uint8_t RF24_GetStatus(RF24_Handle_t* dev){
    RF24_WriteRegister(dev, RF24_NOP, &NOP, 0);
    return dev->status;
}

uint8_t RF24_ReadRegister(RF24_Handle_t* dev, uint8_t target, uint8_t* pRxBuffer, uint8_t length){
    beginTransaction(dev);
    uint8_t reg = (R_REGISTER | target);
    spiTransmitReceive(dev->intf, &dev->status, &reg, 1);
    while(length--) spiTransmitReceive(dev->intf, pRxBuffer++, &NOP, 1);
    endTransaction(dev);
    return dev->status;
}

void RF24_WriteRegister(RF24_Handle_t* dev, uint8_t target, const uint8_t* pData, uint8_t length){
    beginTransaction(dev);
    uint8_t reg = (W_REGISTER | target);
    spiTransmitReceive(dev->intf, &dev->status, &reg, 1);
    if(length > 0) spiTransmit(dev->intf, pData, length);
    endTransaction(dev);
}

void RF24_WritePayload(RF24_Handle_t* dev, const void* pData, uint8_t length, uint8_t writeType){

	uint8_t* current = (uint8_t*) pData;
    uint8_t blank = ! length ? 1 : 0;
	uint8_t empty = 0;

    if (!dev->_isDynamic) {
    	length = RF24_MIN(length, dev->payloadSize);
        blank = dev->payloadSize - length;
    }
    else length = RF24_MIN(length, 32);

    beginTransaction(dev);
    spiTransmitReceive(dev->intf, &dev->status, &writeType, 1);
	while (length--) spiTransmit(dev->intf, current++, 1);
	while (blank--) spiTransmit(dev->intf, &empty, 1);
    endTransaction(dev);
}

void RF24_ReadPayload(RF24_Handle_t* dev, void* pBuffer, uint8_t length){

	uint8_t* current = (uint8_t*) pBuffer;
    uint8_t blank = 0;
    uint8_t temp = R_RX_PAYLOAD;

    if (!dev->_isDynamic) {
    	length = RF24_MIN(length, dev->payloadSize);
        blank = (uint8_t)(dev->payloadSize - length);
    }
    else length = RF24_MIN(length, 32);

    beginTransaction(dev);
    spiTransmitReceive(dev->intf, &dev->status, &temp, 1);
    while (length--) spiTransmitReceive(dev->intf, current++, &NOP, 1);
    while (blank--) spiTransmit(dev->intf, &NOP, 1);
    endTransaction(dev);
}

void RF24_SetChannel(RF24_Handle_t* dev, uint8_t channel){
    const uint8_t max_channel = 125;
    uint8_t ch = RF24_MIN(channel, max_channel);
    RF24_WriteRegister(dev, RF_CH, &ch, 1);
}

void RF24_SetPayloadSize(RF24_Handle_t* dev, uint8_t size){
    dev->payloadSize = (uint8_t)(RF24_MAX(1, RF24_MIN(32, size)));

    for (uint8_t i = 0; i < 6; ++i) {
        RF24_WriteRegister(dev, (uint8_t)(RX_PW_P0 + i), &dev->payloadSize, 1);
    }
}

uint8_t RF24_GetPayloadSize(RF24_Handle_t* dev){
    return dev->payloadSize;
}

void RF24_SetAddressWidth(RF24_Handle_t* dev, uint8_t a_width){
    a_width = (uint8_t)(a_width - 2);
    if (a_width > 0) {
    	uint8_t temp = (a_width % 4);
    	RF24_WriteRegister(dev, SETUP_AW, &temp,1);
        dev->addrWidth = ((a_width % 4) + 2);
    }
    else{
    	uint8_t temp = 0;
    	RF24_WriteRegister(dev, SETUP_AW, &temp, 1);
        dev->addrWidth = 2;
    }
}

void RF24_SetPALevel(RF24_Handle_t* dev, uint8_t level, uint8_t lnaEnable){
	uint8_t setup;
	RF24_ReadRegister(dev, RF_SETUP, &setup, 1);
    setup  = setup & (0xF8);
    setup |= (((level > RF24_PA_MAX ? (RF24_PA_MAX) : level) << 1) + lnaEnable);
    RF24_WriteRegister(dev, RF_SETUP, &setup, 1);
}

uint8_t RF24_GetPALevel(RF24_Handle_t* dev){
	uint8_t setup;
	RF24_ReadRegister(dev, RF_SETUP, &setup, 1);
    return (setup & (_BV(RF_PWR_LOW) | _BV(RF_PWR_HIGH))) >> 1;
}

uint8_t RF24_Available(RF24_Handle_t* dev){
    uint8_t pipe = 0;
    return RF24_PipeAvailable(dev, &pipe);
}

uint8_t RF24_PipeAvailable(RF24_Handle_t* dev, uint8_t* pipe_num){
    uint8_t pipe = (RF24_GetStatus(dev) >> RX_P_NO) & 0x07;
    if (pipe > 5) return RF24_FALSE;
    
    *pipe_num = pipe;
    return RF24_TRUE;
}

void RF24_SetRetries(RF24_Handle_t* dev, uint8_t delayms, uint8_t count){
	uint8_t temp = (uint8_t)(RF24_MIN(15, delayms) << ARD | RF24_MIN(15, count));
	RF24_WriteRegister(dev, SETUP_RETR, &temp, 1);
}

uint8_t dataRateRegister(RF24_Handle_t* dev, RF24_DataRate_e speed)
{
    dev->txDelay = 85;
    if (speed == RF24_250KBPS) {
         dev->txDelay = 155;
        return _BV(RF_DR_LOW);
    }
    else if (speed == RF24_2MBPS) {
         dev->txDelay = 65;
        return _BV(RF_DR_HIGH);
    }
    return 0; /* RF24 1MBPS */
}

uint8_t RF24_SetDataRate(RF24_Handle_t* dev, RF24_DataRate_e speed){
    uint8_t result = 0;
    uint8_t setup = 0;

    RF24_ReadRegister(dev, RF_SETUP, &setup, 1);

    setup = (uint8_t)(setup & ~(_BV(RF_DR_LOW) | _BV(RF_DR_HIGH)));
    setup |= dataRateRegister(dev, speed);

    RF24_WriteRegister(dev, RF_SETUP, &setup,1);
    RF24_ReadRegister(dev, RF_SETUP, &result, 1);

    return (result == setup);
}

void toggleFeatures(RF24_Handle_t* dev){
	beginTransaction(dev);
	uint8_t reg = ACTIVATE;
	spiTransmitReceive(dev->intf, &reg, &dev->status, 1);
	reg = 0x73;
	spiTransmit(dev->intf, &reg, 1);
	endTransaction(dev);
}

void RF24_PowerUp(RF24_Handle_t* dev){
    if (!(dev->configReg & _BV(PWR_UP))) {
        dev->configReg |= _BV(PWR_UP);
        RF24_WriteRegister(dev, NRF_CONFIG, &dev->configReg, 1);
        delay(RF24_POWERUP_DELAY);
    }
}

uint8_t flush_rx(RF24_Handle_t* dev){
	RF24_WriteRegister(dev, FLUSH_RX, &NOP, 0);
    return dev->status;
}

uint8_t flush_tx(RF24_Handle_t* dev){
	RF24_WriteRegister(dev, FLUSH_TX, &NOP, 0);
    return dev->status;
}

uint8_t RF24_Begin(RF24_Handle_t* dev){

    pinWrite(dev->ce, RF24_PIN_LOW);
    pinWrite(dev->cs, RF24_PIN_HIGH);

	delay(5);

	RF24_SetRetries(dev, 5, 15);
	RF24_SetDataRate(dev, RF24_1MBPS);

	uint8_t before_toggle = 0;
	RF24_ReadRegister(dev, FEATURE, &before_toggle, 1);
	toggleFeatures(dev);
	uint8_t after_toggle = 0;
	RF24_ReadRegister(dev, FEATURE, &after_toggle, 1);

	dev->_isPlus = before_toggle == after_toggle;

	uint8_t temp = 0;
	if (after_toggle) {
		if (dev->_isPlus) {
			toggleFeatures(dev);
		}
		RF24_WriteRegister(dev, FEATURE, &temp, 1);
	}

	RF24_WriteRegister(dev, DYNPD, &temp, 1);
	dev->_isDynamic = 0;

	temp = 0x3F;
	RF24_WriteRegister(dev, EN_AA, &temp, 1);

	temp = 0x03;
	RF24_WriteRegister(dev, EN_RXADDR, &temp, 1);

	RF24_SetPayloadSize(dev, 32);
	RF24_SetAddressWidth(dev, 5);
	RF24_SetChannel(dev, 76);

	temp = _BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT);
	RF24_WriteRegister(dev, NRF_STATUS, &temp,1);

	flush_rx(dev);
	flush_tx(dev);

	temp = (_BV(EN_CRC) | _BV(CRCO));
	RF24_WriteRegister(dev, NRF_CONFIG, &temp, 1);
	RF24_ReadRegister(dev, NRF_CONFIG, &dev->configReg, 1);

	RF24_PowerUp(dev);

	return dev->configReg == (_BV(EN_CRC) | _BV(CRCO) | _BV(PWR_UP)) ? 1 : 0;
}

void RF24_OpenReadingPipe(RF24_Handle_t* dev, uint8_t child, uint8_t* address){
    if (child == 0) {
        memcpy(dev->pipe0addr, address, dev->addrWidth);
        dev->_isPipe0Rx = 1;
    }

    if (child <= 5) {
        if (child < 2) RF24_WriteRegister(dev, child_pipe[child], address, dev->addrWidth);
        else RF24_WriteRegister(dev, child_pipe[child], address, 1);
        uint8_t temp = 0;
        RF24_ReadRegister(dev, EN_RXADDR, &temp, 1);
        temp |= _BV(child_pipe_enable[child]);

        RF24_WriteRegister(dev, EN_RXADDR, &temp, 1);
    }
}

void RF24_CloseReadingPipe(RF24_Handle_t* dev, uint8_t pipe){
	uint8_t temp = 0;
	RF24_ReadRegister(dev, EN_RXADDR, &temp, 1);
    temp &= ~_BV(child_pipe_enable[pipe]);
    RF24_WriteRegister(dev, EN_RXADDR, &temp, 1);
    if (!pipe) dev->_isPipe0Rx = 0;
}

void RF24_OpenWritingPipe(RF24_Handle_t* dev, uint8_t* address){
	RF24_WriteRegister(dev, RX_ADDR_P0, address, dev->addrWidth);
	RF24_WriteRegister(dev, TX_ADDR, address, dev->addrWidth);
}

void RF24_StartListening(RF24_Handle_t* dev){
    RF24_PowerUp(dev);
    dev->configReg |= _BV(PRIM_RX);
    RF24_WriteRegister(dev, NRF_CONFIG, &dev->configReg, 1);
	uint8_t temp = _BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT);
    RF24_WriteRegister(dev, NRF_STATUS, &temp, 1);
    pinWrite(dev->ce, RF24_PIN_HIGH);
    if (dev->_isPipe0Rx) RF24_WriteRegister(dev, RX_ADDR_P0, dev->pipe0addr, dev->addrWidth);
    else RF24_CloseReadingPipe(dev, 0);    
}

void RF24_StopListening(RF24_Handle_t* dev){
    pinWrite(dev->ce, RF24_PIN_LOW);
    delay(dev->txDelay);
	dev->configReg = dev->configReg & ~_BV(PRIM_RX);
	RF24_WriteRegister(dev, NRF_CONFIG, &dev->configReg, 1);
	uint8_t temp = 0;
	RF24_ReadRegister(dev, EN_RXADDR, &temp, 1);
	temp |= _BV(child_pipe_enable[0]);
	RF24_WriteRegister(dev, EN_RXADDR, &temp, 1);
}

void RF24_Read(RF24_Handle_t* dev, void* pBuffer, uint8_t length){
    RF24_ReadPayload(dev, pBuffer, length);
    uint8_t temp = (uint8_t)_BV(RX_DR);
    RF24_WriteRegister(dev, NRF_STATUS, &temp, 1);
}

uint8_t RF24_Write(RF24_Handle_t* dev, const void* pBuffer, uint8_t length){

    RF24_WritePayload(dev, pBuffer, length, W_TX_PAYLOAD);
    pinWrite(dev->ce, RF24_PIN_HIGH);

    uint32_t timer = 0;
    while (!(RF24_GetStatus(dev) & (_BV(TX_DS) | _BV(MAX_RT)))) {
		if (++timer > 95) return RF24_ERROR;
		delay(1);
	}

    pinWrite(dev->ce, RF24_PIN_LOW);

	uint8_t data = (_BV(RX_DR) | _BV(TX_DS) | _BV(MAX_RT));
	RF24_WriteRegister(dev, NRF_STATUS, &data, 1);
	if (dev->status & _BV(MAX_RT)) {
		flush_tx(dev);
		return RF24_ERROR;
	}

	return RF24_OK;
}
