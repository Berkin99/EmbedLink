/*
 *  e32100.h
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

#ifndef E32100_H_
#define E32100_H_

#include <stdint.h>

#define E32100_ERROR    1
#define E32100_OK       0

#define E32100_MAX_BUFFER_LEN         58
#define E32100_MODE_CHANGE_INTERVAL   25
#define E32100_AUX_CHANGE_INTERVAL    2
#define E32100_COMMAND_INTERVAL       5
#define E32100_TIMEOUT                100

#define E32100_DEFAULT_CHANNEL        433
#define E32100_DEFAULT_ADDH           0x00
#define E32100_DEFAULT_ADDL           0x00

#define E32100_DEFAULT_AIRDATARATE    0x05
#define E32100_DEFAULT_BAUDRATE       0x03
#define E32100_DEFAULT_PARITY         0x00

#define E32100_DEFAULT_TXPOWER        0x00
#define E32100_DEFAULT_FEC            0x01
#define E32100_DEFAULT_WAKEUP         0x00
#define E32100_DEFAULT_IOMODE         0x01
#define E32100_DEFAULT_TXMODE         0x00

#define E32100_OFFSET_AIRDATARATE     0
#define E32100_OFFSET_BAUDRATE        3
#define E32100_OFFSET_PARITY          6

#define E32100_OFFSET_TXPOWER         0
#define E32100_OFFSET_FEC             2
#define E32100_OFFSET_WAKEUP          3
#define E32100_OFFSET_IOMODE          6
#define E32100_OFFSET_TXMODE          7

typedef enum {
    E32100_MODE_NORMAL = 0,
    E32100_MODE_WAKEUP,
    E32100_MODE_POWERSAVE,
    E32100_MODE_SLEEP
}E32100_Mode_e;

/*  In sleep mode（mode 3：M1=1, M0=1）, it supports below instructions on list.
 *    Only support 9600 and 8N1 format when set
 */
typedef enum {
    E32100_CMD_SAVE_ON      = 0xC0,
    E32100_CMD_READ_CFG     = 0xC1,
    E32100_CMD_SAVE_OFF     = 0xC2,
    E32100_CMD_MODULE       = 0xC3,
    E32100_CMD_RESET        = 0xC4
}E32100_Command_e;

typedef enum {
    E32100_ADR_300 = 0,
    E32100_ADR_1200,
    E32100_ADR_2400,
    E32100_ADR_4800,
    E32100_ADR_9600,
    E32100_ADR_19200
}E32100_AirDataRate_e;

typedef enum {
    E32100_BAUDRATE_1200 = 0,
    E32100_BAUDRATE_2400,
    E32100_BAUDRATE_4800,
    E32100_BAUDRATE_9600,
    E32100_BAUDRATE_19200,
    E32100_BAUDRATE_38400,
    E32100_BAUDRATE_57600,
    E32100_BAUDRATE_115200
}E32100_BaudRate_e;

typedef enum {
    E32100_PARITY_8N1 = 0,
    E32100_PARITY_8O1,
    E32100_PARITY_8E1
}E32100_UartParity_e;

typedef enum {
    E32100_TXPOWER_20 = 0,
    E32100_TXPOWER_17,
    E32100_TXPOWER_14,
    E32100_TXPOWER_10
}E32100_TxPower_e;

typedef enum {
    E32100_FEC_OFF = 0,
    E32100_FEC_ON
}E32100_Fec_e;

typedef enum {
    E32100_WAKEUP_250 = 0,
    E32100_WAKEUP_500,
    E32100_WAKEUP_750,
    E32100_WAKEUP_1000,
    E32100_WAKEUP_1250,
    E32100_WAKEUP_1500,
    E32100_WAKEUP_1750,
    E32100_WAKEUP_2000
}E32100_WakeUpTime_e;

typedef enum {
    E32100_IOMODE_OFF = 0,
    E32100_IOMODE_ON
}E32100_IOMode_e;

typedef enum {
    E32100_TXMODE_TRANSPARENT = 0,
    E32100_TXMODE_FIXED
}E32100_TXMode_e;

typedef struct E32100_Sped_s{
    E32100_AirDataRate_e airDataRate;
    E32100_BaudRate_e baudRate;
    E32100_UartParity_e parity;
}E32100_Sped_t;

typedef struct E32100_Option_s{
    E32100_TxPower_e txPower;
    E32100_Fec_e fec;
    E32100_WakeUpTime_e wakeUpTime;
    E32100_IOMode_e ioMode;
    E32100_TXMode_e txMode;
}E32100_Option_t;

typedef struct E32100_Config_s{
    uint8_t addh;
    uint8_t addl;
    E32100_Sped_t sped;
    uint16_t channel;
    E32100_Option_t option;
}E32100_Config_t;

/* GPIO Read Pointer */
typedef int8_t (*E32100_PinRead_t)(void* pin);

/* GPIO Write Pointer */
typedef void (*E32100_PinWrite_t)(void* pin, int8_t mode);

/* Read via UART Function Pointer */
typedef int8_t (*E32100_Read_t)(void* pIntf, uint8_t* pRxData, uint16_t len);

/* Write via UART Function Pointer */
typedef int8_t (*E32100_Write_t)(void* pIntf, const uint8_t* pTxData, uint16_t len);

/* Delay Milliseconds Function Pointer */
typedef void (*E32100_Delay_t)(uint32_t ms);

/* E32100 Object */
typedef struct E32100_Device_s{
    E32100_Mode_e mode;
    void* pIntf;
    void* M0;
    void* M1;
    void* AUX;
    E32100_PinRead_t pinRead;
    E32100_PinWrite_t pinWrite;
    E32100_Read_t  read;
    E32100_Write_t write;
    E32100_Delay_t delay;
}E32100_Device_t;

/*
 *  @brief This API creates a new handle for the E32100 module.
 *
 *  @param[in] pIntf  : Pointer to the interface.
 *  @param[in] pinM0  : Pin M0 for mode control.
 *  @param[in] pinM1  : Pin M1 for mode control.
 *  @param[in] pinAUX : AUX pin used for checking module status.
 *  @param[in] setf   : Function pointer to digital write pin state.
 *  @param[in] getf   : Function pointer to digital read pin state.
 *  @param[in] readf  : Function pointer to read data from the module.
 *  @param[in] writef : Function pointer to write data to the module.
 *  @param[in] delayf : Function pointer to introduce delays.
 *
 *  @return A new E32100_Device_t structure initialized with the provided parameters.
 */
E32100_Device_t E32100_NewDevice(void* pIntf, void* pinM0, void* pinM1, void* pinAUX,
    E32100_PinRead_t getf, E32100_PinWrite_t setf, E32100_Read_t readf, E32100_Write_t writef, E32100_Delay_t delayf);

/*
 *  @brief This API initializes the E32100 module by setting it to normal mode.
 *
 *  @param[in] self  : Pointer to the E32100 handle.
 *
 *  @return None
 */
void E32100_Init(E32100_Device_t* self);

/*
 *  @brief This API tests the connection by checking the AUX pin.
 *
 *  @param[in] self  : Pointer to the E32100 handle.
 *
 *  @retval 0  -> Success (AUX pin is set)
 *  @retval -1 -> Error (AUX pin is not set)
 */
int8_t E32100_TestConnection(E32100_Device_t* self);

/*
 *  @brief This API sets the module's mode using the M0 and M1 pins.
 *
 *  @param[in] self  : Pointer to the E32100 handle.
 *  @param[in] mode  : Desired mode to set (normal, sleep, etc.).
 *
 *  @return None
 */
void E32100_SetMode(E32100_Device_t* self, E32100_Mode_e mode);

/*
 *  @brief This API configures the module with the specified settings and saves them if required.
 *
 *  @param[in] self   : Pointer to the E32100 handle.
 *  @param[in] config : Configuration structure with settings.
 *  @param[in] save   : Flag indicating whether to save the settings (1 to save, 0 otherwise).
 *
 *  @return None
 */
void E32100_SetConfig(E32100_Device_t* self, E32100_Config_t config, uint8_t save);

/*
 *  @brief This API retrieves the default configuration for the module.
 *
 *  @return The default E32100_Config_t configuration structure.
 */
E32100_Config_t E32100_GetDefaultConfig(void);

/*
 *  @brief This API sets the module to its default configuration.
 *
 *  @param[in] self  : Pointer to the E32100 handle.
 *  @param[in] save  : Flag indicating whether to save the settings (1 to save, 0 otherwise).
 *
 *  @return None
 */
void E32100_SetDefaultConfig(E32100_Device_t* self, uint8_t save);

/*
 *  @brief This API sends a command to the module.
 *
 *  @param[in] self  : Pointer to the E32100 handle.
 *  @param[in] cmd   : Command to send to the module.
 *
 *  @return None
 */
void E32100_Command(E32100_Device_t* self, E32100_Command_e cmd);

/*
 *  @brief This API resets the E32100 module and sets it to normal mode.
 *
 *  @param[in] self  : Pointer to the E32100 handle.
 *
 *  @return None
 */
void E32100_Reset(E32100_Device_t* self);

/*
 *  @brief This API retrieves the current configuration from the module.
 *
 *  @param[in] self   : Pointer to the E32100 handle.
 *  @param[out] buffer: Pointer to the buffer where the configuration will be stored.
 *
 *  @return None
 */
void E32100_GetConfig(E32100_Device_t* self, uint8_t* buffer);

/*
 *  @brief This API retrieves the module's version information.
 *
 *  @param[in] self   : Pointer to the E32100 handle.
 *  @param[out] buffer: Pointer to the buffer where the version information will be stored.
 *
 *  @return None
 */
void E32100_GetModuleVersion(E32100_Device_t* self, uint8_t* buffer);

/*
 *  @brief This API waits for the AUX pin to be ready within a specified timeout.
 *
 *  @param[in] self    : Pointer to the E32100 handle.
 *  @param[in] timeout : Timeout duration in milliseconds.
 *
 *  @return None
 */
void E32100_WaitAUX(E32100_Device_t* self, uint16_t timeout);

/*
 *  @brief This API writes data to the module, checking the AUX pin before transmission.
 *
 *  @param[in] self    : Pointer to the E32100 handle.
 *  @param[in] pTxData : Pointer to the data to be transmitted.
 *  @param[in] size    : Size of the data to be transmitted.
 *
 *  @retval 0  -> Success
 *  @retval >0 -> Warning or Error
 */
int8_t E32100_Write(E32100_Device_t* self, const uint8_t* pTxData, uint16_t size);

/*
 *  @brief This API reads data from the module.
 *
 *  @param[in] self    : Pointer to the E32100 handle.
 *  @param[out] pRxData: Pointer to the buffer where the received data will be stored.
 *  @param[in] size    : Size of the data to be read.
 *
 *  @retval 0  -> Success
 *  @retval >0 -> Warning or Error
 */
int8_t E32100_Read(E32100_Device_t* self, uint8_t *pRxData, uint16_t size);

/*
 *  @brief This API converts the speed configuration into a byte value.
 *
 *  @param[in] sped  : Speed configuration structure.
 *
 *  @return A byte representing the speed configuration.
 */
uint8_t E32100_SpedByte(E32100_Sped_t sped);

/*
 *  @brief This API converts the option configuration into a byte value.
 *
 *  @param[in] option : Option configuration structure.
 *
 *  @return A byte representing the option configuration.
 */
uint8_t E32100_OptionByte(E32100_Option_t option);

/*
 *  @brief This API converts the communication channel into a byte value.
 *
 *  @param[in] channel : Communication channel number.
 *
 *  @return A byte representing the communication channel.
 */
uint8_t E32100_ChannelByte(uint16_t channel);

#endif /* E32100_H_ */
