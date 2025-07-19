
#include "ublox.h"
#include <string.h>

#define UBLOX_REFRATE_DATA_LEN 	 	14
#define UBLOX_CMD_DATA_LEN 		 	8

#define RECEIVE_BUFFER_LEN			256
#define RECEIVE_DEVICE_ID_LEN		17
#define RECEIVE_NAV_DATA_LEN		28
#define RECEIVE_POSLLH_DATA_LEN		36	
#define RECEIVE_PVT_DATA_LEN		100

/* 31.3.1.1 Set protocols and baud rate.
 * PUBX-CONFIG
 * */
typedef struct UBLOX_BaudRateData_s {
    uint8_t UBLOX_baudrateIndex;
    const char *ubxMsg;
} UBLOX_BaudRateData_t;

static const UBLOX_BaudRateData_t UBLOX_BaudRateData[] = {
	{UBLOX_BAUD_RATE_9600,   "$PUBX,41,1,0003,0003,9600,0*14\r\n"  },
	{UBLOX_BAUD_RATE_19200,  "$PUBX,41,1,0003,0003,19200,0*21\r\n" },
	{UBLOX_BAUD_RATE_38400,  "$PUBX,41,1,0003,0003,38400,0*24\r\n" },
	{UBLOX_BAUD_RATE_57600,  "$PUBX,41,1,0003,0003,57600,0*2F\r\n" },
	{UBLOX_BAUD_RATE_115200, "$PUBX,41,1,0003,0003,115200,0*1C\r\n"}
};

/* 32.10.23.1 Navigation/measurement rate settings.
 * UBX-CFG-RATE
 * */
typedef struct UBLOX_RefreshRateData_s{
	uint8_t UBLOX_refreshRateIndex;
	uint8_t ubxMsg[14];
}UBLOX_RefreshRateData_t;

static UBLOX_RefreshRateData_t UBLOX_RefreshRateData[] = {
	{UBLOX_REFRESH_RATE_1HZ, 	{0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xE8, 0x03, 0x01, 0x00, 0x01, 0x00, 0x01, 0x39}},
	{UBLOX_REFRESH_RATE_2HZ, 	{0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x01, 0xF4, 0x01, 0x00, 0x01, 0x00, 0x0B, 0x84}},
	{UBLOX_REFRESH_RATE_5HZ, 	{0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00, 0x01, 0x00, 0xDE, 0x6A}},
	{UBLOX_REFRESH_RATE_10HZ, 	{0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x64, 0x00, 0x01, 0x00, 0x01, 0x00, 0x7A, 0x12}},
	{UBLOX_REFRESH_RATE_20HZ, 	{0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x32, 0x00, 0x01, 0x00, 0x01, 0x00, 0x48, 0xE6}}
};

/* UBX Various Commands : @UBLOX_Command_e */
static uint8_t UBLOX_CMDData[][8]={
	{0xB5, 0x62, 0x27, 0x03, 0x00, 0x00, 0x2A, 0xA5},
	{0xB5, 0x62, 0x01, 0x21, 0x00, 0x00, 0x22, 0x67},
	{0xB5, 0x62, 0x01, 0x02, 0x00, 0x00, 0x03, 0x0A},
	{0xB5, 0x62, 0x01, 0x07, 0x00, 0x00, 0x08, 0x19},
	{0xB5, 0x62, 0x0A, 0x04, 0x00, 0x00, 0x0E, 0x34},
};

static uint8_t UBLOX_CMD_DisableGLL[11] 	= {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x01, 0x00, 0xFB, 0x11};
static uint8_t UBLOX_CMD_DisableGSA[11] 	= {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x02, 0x00, 0xFC, 0x13};
static uint8_t UBLOX_CMD_DisableGSV[11] 	= {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x03, 0x00, 0xFD, 0x15};
static uint8_t UBLOX_CMD_DisableRMC[11] 	= {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x04, 0x00, 0xFE, 0x17};
static uint8_t UBLOX_CMD_DisableVTG[11] 	= {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x05, 0x00, 0xFF, 0x19};
static uint8_t UBLOX_CMD_EnableGGA [11]	    = {0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x00, 0x01, 0xFB, 0x10};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

UBLOX_Handle_t UBLOX_Init(void* intf, ubxWrite writef, ubxRead readf, ubxDelayMs delayf){
	UBLOX_Handle_t temp;
    temp.write = writef;
    temp.read = readf;
    temp.delay = delayf;
	temp.intf = intf;
	return temp;
}

/* UBLOX_LoadConfig called once at initializion routine.
 * Overwrite for custom config.
 */
void UBLOX_LoadConfig(UBLOX_Handle_t* dev){
	/* Baud Config Needed First ! */

	/* MSG Config UART1 */
	dev->write(dev->intf, UBLOX_CMD_DisableGLL, 11); /* GLL Frequency 0 */
	dev->delay(UBLOX_CONFIG_CHANGE_INTERVAL);
	dev->write(dev->intf, UBLOX_CMD_DisableGSA, 11); /* GSA Frequency 0 */
	dev->delay(UBLOX_CONFIG_CHANGE_INTERVAL);
	dev->write(dev->intf, UBLOX_CMD_DisableGSV, 11); /* GSV Frequency 0 */
	dev->delay(UBLOX_CONFIG_CHANGE_INTERVAL);
	dev->write(dev->intf, UBLOX_CMD_DisableRMC, 11); /* RMC Frequency 0 */
	dev->delay(UBLOX_CONFIG_CHANGE_INTERVAL);
	dev->write(dev->intf, UBLOX_CMD_DisableVTG, 11); /* VTG Frequency 0 */
	dev->delay(UBLOX_CONFIG_CHANGE_INTERVAL);
	dev->write(dev->intf, UBLOX_CMD_EnableGGA,  11); /* GGA Frequency 5 */
	dev->delay(UBLOX_CONFIG_CHANGE_INTERVAL);
	UBLOX_SetRefreshRate(dev, UBLOX_REFRESH_RATE_5HZ); /* Refresh Rate 5 Hz */
	dev->delay(UBLOX_CONFIG_CHANGE_INTERVAL);
}

uint8_t UBLOX_Test(UBLOX_Handle_t* dev){
	return UBLOX_UniqueID(dev);
}

// uint8_t UBLOX_ParseNMEA(UBLOX_Handle_t* dev , uint8_t* buffer, uint16_t len){
// 	/* Slice each sentence by "$" ... "<CR><LF>" */
// 	NMEA_Message_t nmea_msg;

// 	if(*buffer!='$')return 0;
// 	if(!NMEA_Pack(&nmea_msg,  buffer))return 0;

// 	switch(nmea_msg.payloadId){
// 	case NMEA_MSG_GGA:{
// 		NMEA_Payload_GGA_t temp;
// 		if(NMEA_GGA_Parse(&temp, &nmea_msg)){
// 			dev->time 		 = temp.time;
// 			dev->location    = temp.location;
// 			dev->quality 	 = temp.quality;
// 			dev->satellite_n = temp.satellite_n;
// 			return 1;
// 		}
// 	}break;
// 	/* CASE */
// 	default:break;
// 	}
// 	return 0;
// }

void UBLOX_SetBaudRate(UBLOX_Handle_t* dev, UBLOX_BaudRate_e rate){
	dev->baudRate = rate;
	dev->write(dev->intf, (uint8_t *)UBLOX_BaudRateData[rate].ubxMsg, strlen(UBLOX_BaudRateData[rate].ubxMsg));
}

void UBLOX_SetRefreshRate(UBLOX_Handle_t* dev, UBLOX_RefreshRate_e rate){
	dev->refreshRate = rate;
	dev->write(dev->intf, UBLOX_RefreshRateData[rate].ubxMsg, UBLOX_REFRATE_DATA_LEN);
}

void UBLOX_TransmitCMD(UBLOX_Handle_t* dev, UBLOX_Command_e index){
	dev->write(dev->intf, UBLOX_CMDData[index], UBLOX_CMD_DATA_LEN);
}

uint8_t UBLOX_UniqueID(UBLOX_Handle_t* dev){
	UBLOX_TransmitCMD(dev, UBLOX_CMD_DEVICE_ID);

	uint8_t buffer[15];
	dev->read(dev->intf, buffer, 15);
	
	for (int i = 0; i < 5; ++i) {
		dev->uniqueID[i] = buffer[10 + i];
	}

	return 1;
}
