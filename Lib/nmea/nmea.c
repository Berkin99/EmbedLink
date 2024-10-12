/*
 *	nmea.c
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
 */

#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdarg.h>

#include "nmea.h"

#define NMEA_MAX_MESSAGE_LEN  	80
#define NMEA_MAX_FIELD_LEN		16

#define NMEA_TALKER_ID_LEN  	2
#define NMEA_PAYLOAD_ID_LEN 	3

#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))


typedef struct NMEA_Identifier_s {
	uint8_t id_index;
	const char* id;
} NMEA_Identifer_t;

static const NMEA_Identifer_t TalkerID_Data[] = {
	{NMEA_TALKER_GP, "GP"},
	{NMEA_TALKER_GL, "GL"},
	{NMEA_TALKER_GA, "GA"},
	{NMEA_TALKER_GB, "GB"},
	{NMEA_TALKER_GN, "GN"},
};

static const NMEA_Identifer_t PayloadID_Data[] = {
	{NMEA_MSG_DTM, "DTM"},
	{NMEA_MSG_GBQ, "GBQ"},
	{NMEA_MSG_GBS, "GBS"}, // Has NMEA Parser
	{NMEA_MSG_GGA, "GGA"}, // Has NMEA Parser
	{NMEA_MSG_GLL, "GLL"}, // Has NMEA Parser
	{NMEA_MSG_GLQ, "GLQ"},
	{NMEA_MSG_GNQ, "GNQ"},
	{NMEA_MSG_GNS, "GNS"},
	{NMEA_MSG_GPQ, "GPQ"},
	{NMEA_MSG_GRS, "GRS"},
	{NMEA_MSG_GSA, "GSA"}, // Has NMEA Parser
	{NMEA_MSG_GST, "GST"}, // Has NMEA Parser
	{NMEA_MSG_GSV, "GSV"}, // Has NMEA Parser
	{NMEA_MSG_RMC, "RMC"}, // Has NMEA Parser
	{NMEA_MSG_TXT, "TXT"},
	{NMEA_MSG_VLW, "VLW"},
	{NMEA_MSG_VTG, "VTG"}, // Has NMEA Parser
	{NMEA_MSG_ZDA, "ZDA"}, // Has NMEA Parser
};

static char* main_cursor;
static uint16_t main_index;

static char talkerid[2];
static char payloadid[3];

bool NMEA_Pack(NMEA_Message_t* ref, const uint8_t* raw) {

	if (*raw != '$') return false;

	main_index = 1;
	uint8_t i;

	for (i = 0; i < NMEA_TALKER_ID_LEN; i++) {
		talkerid[i] = raw[main_index];
		main_index++;
	}

	for (i = 0; i < NMEA_PAYLOAD_ID_LEN; i++) {
		payloadid[i] = raw[main_index];
		main_index++;
	}


	ref->rawdata = (uint8_t*)raw;
	ref->talkerId = NMEA_Find_TalkerID(talkerid);
	ref->payloadId = NMEA_Find_PayloadID(payloadid);
	ref->payload = (uint8_t*)&raw[main_index];

	return true;
}

uint8_t NMEA_Checksum(const char* msg)
{
	/* Support senteces with or without the starting dollar sign. */
	if (*msg == '$') msg++;

	uint8_t checksum = 0x00;

	/*  The optional checksum is an XOR of all bytes between "$" and "*". */
	while (*msg && (*msg != '*')) checksum ^= *msg++;

	return checksum;
}

uint8_t NMEA_Find_TalkerID(const char* msg) {
	uint8_t s_id = ARRAY_SIZE(TalkerID_Data);
	for (uint8_t i = 0; i < s_id; i++) {
		if (strcmp(msg, TalkerID_Data[i].id) == 0) {
			return TalkerID_Data[i].id_index;
		}
	}
	return 0;
}

uint8_t NMEA_Find_PayloadID(const char* msg) {
	uint8_t s_id = ARRAY_SIZE(PayloadID_Data);
	for (uint8_t i = 0; i < s_id; i++) {
		if (strcmp(msg, PayloadID_Data[i].id) == 0) {
			return PayloadID_Data[i].id_index;
		}
	}
	return 0;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/**
* Carries main_cursor to the next ',' value of field.
* @param main_index = Counts move iterations.
* Returns 1 if there is a field.
* Returns 0 if found <message end symbol> or <field len overflow>.
*/
bool NMEA_NextField(const char* msg) {
	for (main_index = 1; main_index < NMEA_MAX_FIELD_LEN; main_index++) {
		if (msg[main_index] == ',') {
			main_cursor = (char*)&msg[main_index];
			return true;
		}
		if (msg[main_index] == '*') {
			main_cursor = (char*)&msg[main_index];
			return false;
		}
	}
	return false;
}



/**
 * Scanf-like processor for NMEA sentences. Supports the following formats:
 * c - single character (char *)
 * d - signed decimal number (int32_t *)
 * f - signed fractional number (double *)
 * u - unsigned decimal, default zero (uint32_t *)
 * i - unsigned byte (uint8_t *)
 * s - string (char *)
 * q - direction N,E = 1 : S,W = -1 (int8_t *)
 * D - date (NMEA_Date *)
 * T - time stamp (NMEA_Time *)
 * L - location (NMEA_Location.latitude *) "latitude,longitude"
 * _ - ignore this field
 * Returns true on success. See library source code for details.
 */
uint8_t NMEA_Scan(const NMEA_Message_t* msg, const char* format, ...) {

	if (msg->payload == NULL) return 0;

	uint8_t result = 0;

	main_cursor = (char*)msg->payload;				// main_cursor[0] points the first ',' element of payload section.

	va_list payload;
	va_start(payload, format);

#define FIELD_CONTROL(cursor) (cursor == ',' || cursor == '*')

#define DIGIT_CONTROL(val) (val>='0' && val<='9')

	while (*format) {
		char type = *format++;				// Get the current format char. && Post increment.

		switch (type)
		{
		case 'c': /* char */ {
			if (FIELD_CONTROL(main_cursor[1])) {
				*va_arg(payload, char*) = ' ';
				break;
			}
			*va_arg(payload, char*) = main_cursor[1];
		} break;

		case 'd': { /* int32_t */
			if (FIELD_CONTROL(main_cursor[1])) {
				*va_arg(payload, int32_t*) = 0;
				break;
			}
			if (!(DIGIT_CONTROL(main_cursor[1]) || main_cursor[1] == '-')) goto parse_error;

			char* ptr;
			*va_arg(payload, int32_t*) = strtol(&main_cursor[1], &ptr, 10);
		} break;

		case 'f': { /* double */
			if (FIELD_CONTROL(main_cursor[1])) {
				*va_arg(payload, float*) = 0;
				break;
			}
			if (!(DIGIT_CONTROL(main_cursor[1]) || main_cursor[1] == '-')) goto parse_error;

			char* ptr;
			*va_arg(payload, float*) = strtod(&main_cursor[1], &ptr);
		} break;

		case ('u'): { /* uint32_t */
			if (FIELD_CONTROL(main_cursor[1])) {
				*va_arg(payload, uint32_t*) = 0;
				break;
			}
			if (!(DIGIT_CONTROL(main_cursor[1]) || main_cursor[1] == '-')) goto parse_error;

			char* ptr;
			*va_arg(payload, uint32_t*) = (uint32_t)strtol(&main_cursor[1], &ptr, 10);
		} break;

		case ('i'): { /* uint8_t */
			if (FIELD_CONTROL(main_cursor[1])) {
				*va_arg(payload, uint8_t*) = 0;
				break;
			}

			if (!(DIGIT_CONTROL(main_cursor[1]) || main_cursor[1] == '-')) goto parse_error;

			char* ptr;
			*va_arg(payload, uint8_t*) = (uint8_t)strtol(&main_cursor[1], &ptr, 10);
		} break;

		case 's': { /* string */
			char* ptr = va_arg(payload, char*);

			uint8_t i = 1;
			while (!FIELD_CONTROL(main_cursor[i])) {
				*ptr++ = main_cursor[i];
				i++;
			}
		} break;

		case 'q': { /* direction int8_t */
			if (FIELD_CONTROL(main_cursor[1])) {
				*va_arg(payload, int8_t*) = 0;
				break;
			}
			int8_t direction = 0;

			switch (main_cursor[1]) {
			case 'N':
			case 'E': {
				direction =  1;
			}break;
			case 'S':
			case 'W': {
				direction = -1;
			}break;

			default: {
				goto parse_error;
			}break;
			}

			*va_arg(payload, int8_t*) = direction;

		} break;

		case 'D': {
			NMEA_Date_t* date_ = va_arg(payload, NMEA_Date_t*);

			if (FIELD_CONTROL(main_cursor[1])) {
				date_->year = -1;
				date_->month = -1;
				date_->day = -1;
				break;
			}

			int16_t y = -1;
			int8_t  m, d = -1;


			char dArr[] = { main_cursor[1], main_cursor[2], '\0' };
			char mArr[] = { main_cursor[3], main_cursor[4], '\0' };
			char yArr[] = { main_cursor[5], main_cursor[6], '\0' };

			d = (int8_t)strtol(dArr, NULL, 10);
			m = (int8_t)strtol(mArr, NULL, 10);
			y = (int16_t)strtol(yArr, NULL, 10);

			y += 2000;

			date_->year = y;
			date_->month = m;
			date_->day = d;

		} break;
		case 'T': {

			NMEA_Time_t* time_ = va_arg(payload, NMEA_Time_t*);

			if (FIELD_CONTROL(main_cursor[1])) {
				time_->hour = -1;
				time_->min = -1;
				time_->sec = -1;
				break;
			}
			int8_t h, m, s = -1;

			char hArr[] = { main_cursor[1], main_cursor[2], '\0' };
			char mArr[] = { main_cursor[3], main_cursor[4], '\0' };
			char sArr[] = { main_cursor[5], main_cursor[6], '\0' };

			h = (int8_t)strtol(hArr, NULL, 10);
			m = (int8_t)strtol(mArr, NULL, 10);
			s = (int8_t)strtol(sArr, NULL, 10);

			time_->hour = h;
			time_->min = m;
			time_->sec = s;

		} break;
		case 'L': { /* location int32_t */
			if (FIELD_CONTROL(main_cursor[1])) {
				*va_arg(payload,int32_t*) = -1;
				break;
			}
			if (!(DIGIT_CONTROL(main_cursor[1]) || main_cursor[1] == '-')) goto parse_error;

			int32_t val, scl = 0;
			int32_t temp = 0;
			char* ptr;
			val = strtol(&main_cursor[1], &ptr, 10);
			scl = strtol(&ptr[1], &ptr, 10);

			temp = (val / 100);
			temp *= 100;

			scl += (val - temp) * 100000;
			scl *= 10;
			scl /= 6;

			temp *= 100000;
			temp += scl;
			*va_arg(payload, int32_t*) = temp;

		} break;
		case 'F': { /* double */
			if (FIELD_CONTROL(main_cursor[1])) {
				*va_arg(payload, double*) = 0;
				break;
			}
			if (!(DIGIT_CONTROL(main_cursor[1]) || main_cursor[1] == '-')) goto parse_error;

			char* ptr;
			*va_arg(payload, double*) = strtod(&main_cursor[1], &ptr);
		} break;
		case '_': { /* Ignore Field */
		}break;
		default: { /* Unknown */
			goto parse_error;
		}break;
		} /* SWITCH_CASE */

		if (!NMEA_NextField(main_cursor)) {
			if (main_cursor[0] == '*') result = 1;
			break;
		}

	}

	result = 1;

parse_error:
	va_end(payload);
	return result;

}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/* GBS GNSS satellite fault detection.
*/
uint8_t NMEA_GBS_Parse(NMEA_Payload_GBS_t* frame, const NMEA_Message_t* msg) {
	if (msg->payloadId != NMEA_MSG_GBS) return 0;

	//$GPGBS,235503.00,1.6,1.4,3.2,,,,,,*40
	//$GPGBS,235458.00,1.4,1.3,3.1,03,,-21.4,3.8,1,0*5B

	return NMEA_Scan(msg, "Tfffdfff",
		&frame->time,
		&frame->errLat,
		&frame->errLon,
		&frame->errAlt,
		&frame->svid,
		&frame->prob,
		&frame->bias,
		&frame->stddev
	);
}

/* GGA Global Positioning System Fix Data. Time, Position and fix related data
*  for GNSS receiver.
*/
uint8_t NMEA_GGA_Parse(NMEA_Payload_GGA_t* frame, const NMEA_Message_t* msg) {
	if (msg->payloadId != NMEA_MSG_GGA) return 0;

	//$GNGGA,092725.00,4717.11399,N,00833.91590,E,1,08,1.01,499.6,M,48.0,M,,*5B

	return NMEA_Scan(msg, "TLqLqii",
		&frame->time,
		&frame->location.latitude,
		&frame->location.ns_d,
		&frame->location.longitude,
		&frame->location.ew_d,
		&frame->quality,
		&frame->satellite_n
	);
}


/* GLL Latitude and longitude, with time of position fix and status.
*/
uint8_t NMEA_GLL_Parse(NMEA_Payload_GLL_t* frame, const NMEA_Message_t* msg) {
	if (msg->payloadId != NMEA_MSG_GLL) return 0;

	//$GPGLL,4717.11364,N,00833.91565,E,092321.00,A,A*60

	return NMEA_Scan(msg, "LqLqTcc",
		&frame->location.latitude,
		&frame->location.ns_d,
		&frame->location.longitude,
		&frame->location.ew_d,
		&frame->time,
		&frame->status,
		&frame->posMode
	);
}

/* GSA  GNSS DOP and active satellites.
*/
uint8_t NMEA_GSA_Parse(NMEA_Payload_GSA_t* frame, const NMEA_Message_t* msg) {
	if (msg->payloadId != NMEA_MSG_GSA) return 0;

	//$GPGSA,A,3,23,29,07,08,09,18,26,28,,,,,1.94,1.18,1.54,1*0D

	return NMEA_Scan(msg, "ciiiiiiiiiiiiifffi",
		&frame->opMode,
		&frame->navMode,
		&frame->sats[0],
		&frame->sats[1],
		&frame->sats[2],
		&frame->sats[3],
		&frame->sats[4],
		&frame->sats[5],
		&frame->sats[6],
		&frame->sats[7],
		&frame->sats[8],
		&frame->sats[9],
		&frame->sats[10],
		&frame->sats[11],
		&frame->pdop,
		&frame->hdop,
		&frame->vdop,
		&frame->fix_type
	);
}

/* GST  GNSS pseudorange error statistics.
*/
uint8_t NMEA_GST_Parse(NMEA_Payload_GST_t* frame, const NMEA_Message_t* msg) {
	if (msg->payloadId != NMEA_MSG_GST) return 0;

	//$GPGST,082356.00,1.8,,,,1.7,1.3,2.2*7E

	return (uint8_t)NMEA_Scan(msg, "Tfffffff",
		&frame->time,
		&frame->rangeRms,
		&frame->stdMajor,
		&frame->stdMinor,
		&frame->orient,
		&frame->stdLat,
		&frame->stdLon,
		&frame->stdAlt
	);
}

/* GSV  GNSS satellites in view.
*/
uint8_t NMEA_GSV_Parse(NMEA_Payload_GSV_t* frame, const NMEA_Message_t* msg) {
	if (msg->payloadId != NMEA_MSG_GSV) return 0;

	//$GPGSV,1,1,03,12,,,42,24,,,47,32,,,37,5*66

	return (uint8_t)NMEA_Scan(msg, "iiddddddddddddddddd",
		&frame->msgNum,
		&frame->numMsg,
		&frame->numSV,
		&frame->sats[0].nr,
		&frame->sats[0].elevation,
		&frame->sats[0].azimuth,
		&frame->sats[0].snr,
		&frame->sats[1].nr,
		&frame->sats[1].elevation,
		&frame->sats[1].azimuth,
		&frame->sats[1].snr,
		&frame->sats[2].nr,
		&frame->sats[2].elevation,
		&frame->sats[2].azimuth,
		&frame->sats[2].snr,
		&frame->sats[3].nr,
		&frame->sats[3].elevation,
		&frame->sats[3].azimuth,
		&frame->sats[3].snr
	);
}

/* RMC Recommended minimum data. (I don't recommend)
*/
uint8_t NMEA_RMC_Parse(NMEA_Payload_RMC_t* frame, const NMEA_Message_t* msg) {
	if (msg->payloadId != NMEA_MSG_RMC) return 0;

	//$GPRMC,083559.00,A,4717.11437,N,00833.91522,E,0.004,77.52,091202,,,A,V*57

	return (uint8_t)NMEA_Scan(msg, "TcLqLqffDf_c",
		&frame->time,
		&frame->status,
		&frame->location.latitude,
		&frame->location.ns_d,
		&frame->location.longitude,
		&frame->location.ew_d,
		&frame->speed,
		&frame->course,
		&frame->date,
		&frame->variation,
		&frame->posMode
	);
}


/* VTG  Course over ground and ground speed.
*/
uint8_t NMEA_VTG_Parse(NMEA_Payload_VTG_t* frame, const NMEA_Message_t* msg) {
	if (msg->payloadId != NMEA_MSG_VTG) return 0;

	//$GPVTG,77.52,T,,M,0.004,N,0.008,K,A*06

	return (uint8_t)NMEA_Scan(msg, "f_f_f_f_c",
		&frame->cogt,
		&frame->cogm,
		&frame->sogn,
		&frame->sogk,
		&frame->posMode
	);
}

/* ZDA Time and Date.
*/
uint8_t NMEA_ZDA_Parse(NMEA_Payload_ZDA_t* frame, const NMEA_Message_t* msg) {
	if (msg->payloadId != NMEA_MSG_ZDA) return 0;

	//$GPZDA,082710.00,16,09,2002,00,00*64

	return (uint8_t)NMEA_Scan(msg, "Tddddd",
		&frame->time,
		&frame->date.day,
		&frame->date.month,
		&frame->date.year,
		&frame->hour_offset,
		&frame->minute_offset
	);
}
