/*
 *  rf24_user_defs.h
 *  
 *  @brief : User should define hardware abstraction functions.
 */

#include <stdint.h>
#include "systime.h"
#include "spi.h"
#include "gpio.h"
#include "sysconfig.h"

#define RF24_PIN_HIGH      HIGH
#define RF24_PIN_LOW       LOW
#define RF24_OK            OK
#define RF24_ERROR         E_ERROR
#define RF24_TRUE          TRUE
#define RF24_FALSE         FALSE

#ifndef SPI_H_
/* 
 * Pin identifiers passed to this function 
 * pin : @RF24_Handle_t/ce RF24_Handle_t/cs
 */
inline void pinWrite (uint16_t pin, uint8_t state){
    /* USER DEFINE */
}

/* @brief : SPI Mutex take. If system has no RTOS: empty  */
inline void  spiBeginTransaction(void* intf){
    /* USER DEFINE */
}

/* @brief : SPI Mutex give. If system has no RTOS: empty  */
inline void spiEndTransaction(void* intf){
    /* USER DEFINE */
}

inline int8_t spiReceive(void* intf, uint8_t* pRxData, uint16_t len){
    /* USER DEFINE */
}

inline int8_t spiTransmit(void* intf, const uint8_t* pTxData, uint16_t len){
    /* USER DEFINE */
}

inline int8_t spiTransmitReceive(void* intf, uint8_t* pRxData, const uint8_t* pTxData, uint16_t len){
    /* USER DEFINE */
}

inline void delay(uint32_t ms){
    /* USER DEFINE */
}
#endif
