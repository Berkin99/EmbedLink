/*
 *	nmea.h
 *
 *  Created on: Dec 13, 2023
 *      Author: BerkN
 *
 *  NMEA 0183 Protocol Parser for Ublox GNSS Receivers.
 *	Easy to use for all NMEA applications / Extendable NMEA message ID library.
 *
 *  13.12.2023 : File is created for read UBLOX GNSS NMEA messages. Detect &
 *  parse GGA lines and return NMEA_GGA_t.
 *
 *	14.12.2023 : NMEA_Pack, detects all valid [1] Ublox_M8 receiver talkerID &
 *  payloadID messages.
 *
 *	16.12.2023 : Usage of generalized scan algorithym.
 *  "kosma/minmea" minmea_scan()
 *	Added GBS, RMC, GLL, GST, GSA, GSV, VTG, ZDA.
 *
 *	18.12.2023 : Compiler satisfy changes.
 *
 *	References:
 *  [0] The National Marine Electronics Association (NMEA) 0183. Manual Klaus Betke, May 2000. Revised August 2001.
 *	[1] u-blox8-M8_ReceiverDescrProtSpec_(UBX-13003221)
 *  [2] github.com/kosma/minmea
 *
 */

#ifndef NMEA_H_
#define NMEA_H_

#include <stdint.h>
#include <string.h>
#include <stdbool.h>

typedef enum {
	NMEA_TALKER_GP = 1,		//GPS, SBAS, QZSS
	NMEA_TALKER_GL,			//GLONASS
	NMEA_TALKER_GA,			//Galileo
	NMEA_TALKER_GB,			//BeiDou
	NMEA_TALKER_GN,			//GNSS Combination
}NMEA_TalkerId_e;

typedef enum {
	NMEA_MSG_DTM = 1,
	NMEA_MSG_GBQ,
	NMEA_MSG_GBS,
	NMEA_MSG_GGA,
	NMEA_MSG_GLL,
	NMEA_MSG_GLQ,
	NMEA_MSG_GNQ,
	NMEA_MSG_GNS,
	NMEA_MSG_GPQ,
	NMEA_MSG_GRS,
	NMEA_MSG_GSA,
	NMEA_MSG_GST,
	NMEA_MSG_GSV,
	NMEA_MSG_RMC,
	NMEA_MSG_TXT,
	NMEA_MSG_VLW,
	NMEA_MSG_VTG,
	NMEA_MSG_ZDA,
}NMEA_PayloadId_e;

typedef struct NMEA_Message_s {
	uint8_t talkerId;
	uint8_t payloadId;
	uint8_t* rawdata;
	uint8_t* payload;
	uint8_t length;
}NMEA_Message_t;

typedef struct NMEA_Date_s {
	int32_t year;
	int32_t month;
	int32_t day;
}NMEA_Date_t;

typedef struct NMEA_Time_s {
	int8_t hour;
	int8_t min;
	int8_t sec;
}NMEA_Time_t;

typedef struct NMEA_SatInfo_s {
	int32_t nr;
	int32_t elevation;
	int32_t azimuth;
	int32_t snr;
}NMEA_SatInfo_t;

typedef struct NMEA_Location_s {
	int32_t latitude;	// Degrees as integer 1 lat ~ 111000 meters /*47.1234567 deg = 471234567*/
	int32_t longitude;	// Degrees as integer 1 lon = cos(lat*deg2rad)* pi / 180
	int8_t  ns_d;
	int8_t  ew_d;
}NMEA_Location_t;

typedef struct NMEA_Payload_GBS_s {
	NMEA_Time_t time;
	float errLat;
	float errLon;
	float errAlt;
	int32_t svid;
	float prob;
	float bias;
	float stddev;
}NMEA_Payload_GBS_t;

typedef struct NMEA_Payload_GGA_s {
	NMEA_Time_t time;
	NMEA_Location_t location;
	uint8_t quality;
	uint8_t satellite_n;
}NMEA_Payload_GGA_t;

typedef struct NMEA_Payload_GLL_s {
	NMEA_Location_t location;
	NMEA_Time_t time;
	char status;
	char posMode;
}NMEA_Payload_GLL_t;

typedef struct NMEA_Payload_GST_s {
	NMEA_Time_t time;
	float rangeRms;
	float stdMajor;
	float stdMinor;
	float orient;
	float stdLat;
	float stdLon;
	float stdAlt;
}NMEA_Payload_GST_t;

typedef struct NMEA_Payload_GSA_s {
	char opMode;
	uint8_t navMode;
	uint8_t fix_type;
	uint8_t sats[12];
	float pdop;
	float hdop;
	float vdop;
}NMEA_Payload_GSA_t;

typedef struct NMEA_Payload_GSV_s {
	uint8_t numMsg;
	uint8_t msgNum;
	int32_t numSV;
	NMEA_SatInfo_t sats[4];
}NMEA_Payload_GSV_t;

typedef struct NMEA_Payload_RMC_s {
	NMEA_Time_t time;
	char status;
	NMEA_Location_t location;
	float speed;
	float course;
	NMEA_Date_t date;
	float variation;
	char posMode;
	char navStatus;
}NMEA_Payload_RMC_t;

typedef struct NMEA_Payload_VTG_s {
	float cogt;
	float cogm;
	float sogn;
	float sogk;
	char posMode;
}NMEA_Payload_VTG_t;

typedef struct NMEA_Payload_ZDA_s {
	NMEA_Time_t time;
	NMEA_Date_t date;
	int32_t hour_offset;
	int32_t minute_offset;
}NMEA_Payload_ZDA_t;

uint8_t NMEA_Pack(NMEA_Message_t* ref, const uint8_t* raw_sentence);

uint8_t NMEA_Find_TalkerID(const char* msg);

uint8_t NMEA_Find_PayloadID(const char* msg);

/**
 * Scanf-like processor for NMEA sentences. Supports the following formats:
 * c - single character (char *)
 * d - signed decimal number (int32_t *)
 * f - signed fractional number (double *)
 * u - unsigned decimal, default zero (uint32_t *)
 * i - unsigned byte (uint8_t *)
 * s - string (char *)
 * q - direction N,E = 1 : S,W = -1 (int8_t *)
 * D - date (NMEA_Date_t *)
 * T - time stamp (NMEA_Time_t *)
 * L - location (NMEA_Location_t.latitude *) "latitude,longitude"
 * _ - ignore this field
 * Returns true on success. See library source code for details.
 */
uint8_t NMEA_Scan(const NMEA_Message_t* msg, const char* format, ...);

uint8_t NMEA_Parse_GBS(NMEA_Payload_GBS_t* frame, const NMEA_Message_t* msg);

uint8_t NMEA_Parse_GGA(NMEA_Payload_GGA_t* frame, const NMEA_Message_t* msg);

uint8_t NMEA_Parse_GLL(NMEA_Payload_GLL_t* frame, const NMEA_Message_t* msg);

uint8_t NMEA_Parse_GSA(NMEA_Payload_GSA_t* frame, const NMEA_Message_t* msg);

uint8_t NMEA_Parse_GST(NMEA_Payload_GST_t* frame, const NMEA_Message_t* msg);

uint8_t NMEA_Parse_GSV(NMEA_Payload_GSV_t* frame, const NMEA_Message_t* msg);

uint8_t NMEA_Parse_RMC(NMEA_Payload_RMC_t* frame, const NMEA_Message_t* msg);

uint8_t NMEA_Parse_VTG(NMEA_Payload_VTG_t* frame, const NMEA_Message_t* msg);

uint8_t NMEA_Parse_ZDA(NMEA_Payload_ZDA_t* frame, const NMEA_Message_t* msg);

#endif /* NMEA_H */
