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

#include "ntrp.h"
#include <stdint.h>

uint8_t NTRP_Parse(NTRP_Message_t* ref, const uint8_t* raw_sentence){
	if(raw_sentence[0]!=NTRP_STARTBYTE) return 0;
	ref->talkerID    = raw_sentence[1];
	ref->receiverID  = raw_sentence[2];
	ref->packetsize  = raw_sentence[3];

	if(ref->packetsize<2)return 0;

	ref->packet.header = raw_sentence[4];
	ref->packet.dataID = raw_sentence[5];
	for(uint8_t i = 0; i+2<ref->packetsize;i++){
		ref->packet.data.bytes[i] = raw_sentence[i+6];
	}

	if(raw_sentence[ref->packetsize+4] != NTRP_ENDBYTE) return 0;
	return 1;
}

uint8_t NTRP_PackParse(NTRP_Packet_t* ref, const uint8_t* raw_packet){
	ref->header = raw_packet[0];
	ref->dataID = raw_packet[1];

	uint8_t size = NTRP_MAX_PACKET_SIZE-2;
	if(ref->header == NTRP_MSG) size = ref->dataID;

	for(uint8_t i = 0; i<size ;i++){
		ref->data.bytes[i] = raw_packet[i+2];
	}
	return 1;
}

uint8_t NTRP_Unite(uint8_t* ref, const NTRP_Message_t* message){
	ref[0] = NTRP_STARTBYTE;
	ref[1] = message->talkerID;
	ref[2] = message->receiverID;
	ref[3] = message->packetsize;
	uint8_t status = NTRP_PackUnite(&ref[4],message->packetsize,&message->packet);
	ref[message->packetsize+4] = NTRP_ENDBYTE;
	return status;
}

uint8_t NTRP_PackUnite(uint8_t* ref, uint8_t size, const NTRP_Packet_t* packet){
	if(size<2) return 0;
	ref[0] = packet->header;
	ref[1] = packet->dataID;
	for(uint8_t i = 0; i<(size-2) ;i++){
		ref[i+2] = packet->data.bytes[i];
	}
	return 1;
}

