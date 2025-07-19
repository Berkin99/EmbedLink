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

#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include "nrx.h"
#include "nrx_logic.h"
#include "uart.h"

static const uint8_t typeLength[] = {
	[NRX_UINT8]  = 1,
	[NRX_UINT16] = 2,
	[NRX_UINT32] = 4,
	[NRX_INT8]   = 1,
	[NRX_INT16]  = 2,
	[NRX_INT32]  = 4,
	[NRX_FLOAT]  = 4,
};

static nrxVarId_t invalidVarId = {0xffffu, 0xffffu};

/* Set by the Linker */
extern struct nrx_s _nrx_start;
extern struct nrx_s _nrx_stop;

static struct nrx_s *nrxs;
static int nrxsLen;

struct nrx_s* nrxGetVar(uint16_t index){
	if(index>=nrxsLen)return NULL;
	return &nrxs[index];
}

nrxVarId_t nrxGetVarId(const char* group, const char* name){

	uint16_t index;
	uint16_t id = 0;
	nrxVarId_t varId = invalidVarId;
	char * currgroup = "";

	for(index = 0; index < nrxsLen; index++)
	{
		if (nrxs[index].type & NRX_GROUP) {
			  if (nrxs[index].type & NRX_START) {
				currgroup = nrxs[index].name;
			  }
		}
		else id += 1;

		if ((!strcmp(group, currgroup)) && (!strcmp(name, nrxs[index].name))) {
		  varId.index = index;
		  varId.id = id - 1;
		  return varId;
		}
	}

	return invalidVarId;
}

int nrxGetType(uint16_t index){
	return nrxs[index].type;
}

uint8_t nrxVarSize(int type){
	return typeLength[type];
}

uint8_t nrxGroupSize(int index){
	if(index>nrxsLen) return 0;
	if(!(nrxs[index].type & NRX_GROUP) && (nrxs[index].type & NRX_START)) return 0;

	uint8_t bytesize = 0;
	while(1){
		index++;
		if(nrxs[index].type & NRX_GROUP)break;
		bytesize += nrxVarSize(nrxs[index].type);
	}
	return bytesize;
}


void nrxLogicInit(){
	nrxs = &_nrx_start;
	nrxsLen = &_nrx_stop - &_nrx_start;


	/* Print the table */
	#ifdef NRX_DEBUG
	serialPrint("[+] NRX Length : %d\n", nrxsLen);

	char* groupname = nrxs[0].name;
	uint8_t bytesize;

	for(uint16_t i = 0; i<nrxsLen; i++){
		if(nrxs[i].type & NRX_GROUP){
			if(nrxs[i].type & NRX_START){
				groupname = nrxs[i].name;
				bytesize = nrxGroupSize(i);
			}
			else continue;
		}
		else bytesize = nrxVarSize(nrxs[i].type);

		serialPrint("[%d] %s.%s : %d bytes\n",i,groupname,nrxs[i].name,bytesize);
	}
	#endif
}
