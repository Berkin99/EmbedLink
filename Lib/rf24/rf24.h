/*
 *  rf24.h
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

#ifndef RF24_H_
#define RF24_H_

#include <stdint.h>

#define RF24_MAX_PAYLOAD_LENGHT     32
#define RF24_CS_DELAY_MS            5

typedef enum{
    RF24_PA_MIN = 0,
    RF24_PA_LOW,
    RF24_PA_HIGH,
    RF24_PA_MAX,
    RF24_PA_ERROR
} RF24_PA_DBM_e;

typedef enum{
	RF24_1MBPS = 0,
    RF24_2MBPS,
    RF24_250KBPS
} RF24_DataRate_e;

typedef enum{
    RF24_CRC_DISABLED = 0,
    RF24_CRC_8,
    RF24_CRC_16
} RF24_CRCLength_e;

typedef struct RF24_Handle_s{
    uint8_t status;            /* The status byte returned from every SPI transaction */
    uint8_t addrWidth;         /* Address width */
    uint8_t payloadSize;       /* Fixed size of payloads */
    uint8_t pipe0addr[5];      /* Last address set on pipe 0 for reading. */
    uint8_t configReg;         /* NRF_CONFIG register value */
    uint32_t txDelay;          /* Transmit wait interval */
    uint8_t _isPlus;           /* Variant type ? nRF2401 : nRF2401+ */
    uint8_t _isPipe0Rx;        /* Is Pipe 0 in rx mode ? True : False */
    uint8_t _isDynamic;        /* Is Dynamic payloads enabled ? True : False */
    void* intf;                /* Interface pointer : SPI Handle */
    uint16_t ce;               /* GPIO pin identifier */
    uint16_t cs;               /* GPIO pin identifier */
}RF24_Handle_t;

RF24_Handle_t RF24_Init(void* intf, uint16_t ce, uint16_t cs);
uint8_t RF24_GetStatus(RF24_Handle_t* dev);
uint8_t RF24_ReadRegister(RF24_Handle_t* dev, uint8_t target, uint8_t* pRxBuffer, uint8_t length);
void RF24_WriteRegister(RF24_Handle_t* dev, uint8_t target, const uint8_t* pData, uint8_t length);
void RF24_WritePayload(RF24_Handle_t* dev, const void* pData, uint8_t length, uint8_t writeType);
void RF24_ReadPayload(RF24_Handle_t* dev, void* pBuffer, uint8_t length);
void RF24_SetChannel(RF24_Handle_t* dev, uint8_t channel);
void RF24_SetPayloadSize(RF24_Handle_t* dev, uint8_t size);
uint8_t RF24_GetPayloadSize(RF24_Handle_t* dev);
void RF24_SetAddressWidth(RF24_Handle_t* dev, uint8_t a_width);
void RF24_SetPALevel(RF24_Handle_t* dev, uint8_t level, uint8_t lnaEnable);
uint8_t RF24_GetPALevel(RF24_Handle_t* dev);
uint8_t RF24_Available(RF24_Handle_t* dev);
uint8_t RF24_PipeAvailable(RF24_Handle_t* dev, uint8_t* pipe_num);
void RF24_SetRetries(RF24_Handle_t* dev, uint8_t delayms, uint8_t count);
uint8_t RF24_SetDataRate(RF24_Handle_t* dev, RF24_DataRate_e speed);
void RF24_PowerUp(RF24_Handle_t* dev);
uint8_t RF24_Begin(RF24_Handle_t* dev);
void RF24_OpenReadingPipe(RF24_Handle_t* dev, uint8_t child, uint8_t* address);
void RF24_CloseReadingPipe(RF24_Handle_t* dev, uint8_t pipe);
void RF24_OpenWritingPipe(RF24_Handle_t* dev, uint8_t* address);
void RF24_StartListening(RF24_Handle_t* dev);
void RF24_StopListening(RF24_Handle_t* dev);
void RF24_Read(RF24_Handle_t* dev, void* pBuffer, uint8_t length);
uint8_t RF24_Write(RF24_Handle_t* dev, const void* pBuffer, uint8_t length);

#endif
