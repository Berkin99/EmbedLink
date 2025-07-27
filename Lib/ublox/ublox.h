
#ifndef UBLOX_H_
#define UBLOX_H_

#include <stdint.h>

#define UBLOX_TIMEOUT_MS 					4500				// GPS LOST_COMMUNICATION timeout in ms (max time between received nav solutions)
#define UBX_ACK_TIMEOUT_MS 					150					// Timeout for waiting for an ACK or NAK response to a configuration command

#define UBLOX_INIT_INTERVAL 				3000  				// Time to wait, in ms, initialize time
#define UBLOX_CONFIG_CHANGE_INTERVAL 		500       			// Time to wait, in ms, between CONFIG steps
#define UBLOX_BAUDRATE_TEST_COUNT 			3      				// Number of times to repeat the test message when setting baudrate
#define UBLOX_RECV_TIME_MAX 				25           		// Max permitted time, in us, for the Receive Data process

#define UBLOX_BUFFER_SIZE 					256
#define UBLOX_UBX_BUFFER_SIZE				100

#define UBLOX_OK      0
#define UBLOX_ERROR   1

typedef int8_t (*ubxWrite)(void* intf, uint8_t* pTxData, uint16_t len);
typedef int8_t (*ubxRead)(void* intf, uint8_t* pRxData, uint16_t len);
typedef void   (*ubxDelayMs)(uint32_t ms);

typedef enum {
	UBLOX_REFRESH_RATE_1HZ = 0,  		// 1000 ms measurement rate.
	UBLOX_REFRESH_RATE_2HZ,  			// 500 ms measurement rate.
	UBLOX_REFRESH_RATE_5HZ,  			// 200 ms measurement rate.
	UBLOX_REFRESH_RATE_10HZ,	   		// 100 ms measurement rate.
	UBLOX_REFRESH_RATE_20HZ,	   		// 50 ms measurement rate, max for protocol versions less than 24
}UBLOX_RefreshRate_e;

typedef enum {
	UBLOX_BAUD_RATE_9600  = 0,
	UBLOX_BAUD_RATE_19200,
	UBLOX_BAUD_RATE_38400,
	UBLOX_BAUD_RATE_57600,
	UBLOX_BAUD_RATE_115200,
}UBLOX_BaudRate_e;

typedef enum {
	UBLOX_CMD_DEVICE_ID = 0,
	UBLOX_CMD_NAV_DATA,
	UBLOX_CMD_POSLLH_DATA,
	UBLOX_CMD_PVT_DATA,
	UBLOX_CMD_MON_VER
}UBLOX_Command_e;

typedef struct UBLOX_Handle_s{
	void* intf;
    ubxWrite write;
    ubxRead read;
    ubxDelayMs delay;

	UBLOX_BaudRate_e 	baudRate;
	UBLOX_RefreshRate_e refreshRate;
	uint8_t uniqueID [5];

	uint8_t quality;
	uint8_t satellite_n;
}UBLOX_Handle_t;

UBLOX_Handle_t UBLOX_Init(void* intf, ubxWrite writef, ubxRead readf, ubxDelayMs delayf);
uint8_t UBLOX_Test (UBLOX_Handle_t* dev);

void    UBLOX_LoadConfig (UBLOX_Handle_t* dev);
void    UBLOX_SetBaudRate (UBLOX_Handle_t* dev, UBLOX_BaudRate_e rate);
void    UBLOX_SetRefreshRate (UBLOX_Handle_t* dev, UBLOX_RefreshRate_e rate);
void    UBLOX_TransmitCMD (UBLOX_Handle_t* dev, UBLOX_Command_e index);
uint8_t UBLOX_UniqueID (UBLOX_Handle_t* dev);

#endif /* UBLOX_H_ */