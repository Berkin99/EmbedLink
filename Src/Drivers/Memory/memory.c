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
#include "sysconfig.h"
#include "memory.h"
#include "uart.h"

#ifdef E24AA_I2C
#include "memory_e24aa.h"
#endif

#define MEMORY_ADD(NAME) {\
	.Name = memoryName##NAME,\
	.Init = &memoryInit##NAME,\
	.Test = &memoryTest##NAME,\
	.MemRead  = &memoryRead##NAME,\
	.MemWrite = &memoryWrite##NAME,\
},

static const memoryHandle_t mhandleList[] ={
	#ifdef E24AA_I2C
		MEMORY_ADD(E24AA)
	#endif
};

static const uint8_t mhandleLen = sizeof(mhandleList)/sizeof(memoryHandle_t);

static const uint8_t typeLength[] = {
	[MEM_UINT8]  = 1,
	[MEM_UINT16] = 2,
	[MEM_UINT32] = 4,
	[MEM_INT8]   = 1,
	[MEM_INT16]  = 2,
	[MEM_INT32]  = 4,
	[MEM_FLOAT]  = 4,
};

/* Set by the Linker */
extern struct mem_s _mem_start;
extern struct mem_s _mem_stop;
static struct mem_s *mems;
static int memsLen;

const memID_t invalidID = MEM_INVALID_ID;

void memoryInit(void){
	for (uint8_t i = 0; i < mhandleLen; i++) {
        if(mhandleList[i].Init() == OK) serialPrint("[+] Memory %s init OK\n", mhandleList[i].Name);
        else serialPrint("[-] Memory %s init ERROR\n", mhandleList[i].Name);
	}
	memoryLogicInit();
}

void memoryTest(){
	for (uint8_t i = 0; i < mhandleLen; i++) {
        if(mhandleList[i].Test() == OK) serialPrint("[+] Memory %s test OK\n", mhandleList[i].Name);
        else serialPrint("[-] Memory %s test ERROR\n", mhandleList[i].Name);
	}
}

int8_t memoryRead (memory_t key, uint8_t* pRxData, int8_t len){
	return mhandleList[0].MemRead(key, pRxData, len);
}

int8_t memoryWrite(memory_t key, uint8_t* pTxData, int8_t len){
	return mhandleList[0].MemWrite(key, pTxData, len);
}

int8_t memoryClear(void){
	uint32_t lstb = memoryGetKey(memsLen - 2);
	uint32_t tmp = 0;
	int i = 0;
	while(i < lstb){
		uint8_t status = memoryWrite(i, (uint8_t *)&tmp, 4);
		if(status != OK) return status;
		i += 4;
	}
	return OK;
}

int8_t memoryLoad(int8_t(*fptr)(memory_t key, uint8_t* pData, int8_t len)){
	int i;
	memory_t key = 0;
	for (i = 0; i < memsLen; i++) {
		if(mems[i].type & MEM_GROUP) continue;
		uint8_t sz     = memoryTypeSize(mems[i].type);
		uint8_t status = fptr(key, mems[i].address, sz);
		key += sz;
		if(status != OK) return i;
	}
	return OK;
}

int8_t memoryMemLoad(char* group, char* name, int8_t(*fptr)(memory_t key, uint8_t* pData, int8_t len)){
	memID_t id = memoryGetID(group, name);
	if(id.index == MEM_INVALID) return E_NOT_FOUND;
	uint16_t len = 0;
	if(mems[id.index].type & MEM_GROUP){
		id.index++;
		int8_t status = 0;
		while((mems[id.index].type & MEM_GROUP) == 0){
			if(id.index >= memsLen) return E_OVERFLOW;
			len = memoryTypeSize(mems[id.index].type);
			status = fptr(id.key, mems[id.index].address, len);
			if(status != OK) return id.index;
			id.index++;
			id.key += len;
		}
		return OK;
	}

	len = memoryTypeSize(mems[id.index].type);
	return fptr(id.key, mems[id.index].address, len);
}

int8_t memoryUpload(void){return memoryLoad(&memoryWrite);}
int8_t memoryDownload(void){return memoryLoad(&memoryRead);}
int8_t memoryMemUpload  (char* group, char* name){return memoryMemLoad(group, name, &memoryWrite);}
int8_t memoryMemDownload(char* group, char* name){return memoryMemLoad(group, name, &memoryRead);}

mem_t* memoryGetVar(uint16_t index){
	if(index >= memsLen) return NULL;
	return &mems[index];
}

memID_t memoryGetID (char* group, char* name){

	uint16_t i;
	uint32_t key = 0;
	uint8_t  cmp = 1;

	for (i = 0; i < memsLen; i++) {
		if(mems[i].type & MEM_GROUP){
			cmp = strcmp(mems[i].name, group);
			if(cmp == 0) break;
		}
		else key += memoryTypeSize(mems[i].type);
	}

	if(cmp  != 0) return invalidID;
	if(name != NULL){
		i++;
		for (; i < memsLen; i++) {
			if(mems[i].type & MEM_GROUP) return invalidID;
			if(strcmp(mems[i].name, name) == 0) break;
			key += memoryTypeSize(mems[i].type);
		}

	}
	memID_t    id;
	id.index = i;
	id.key   = key;
	return     id;
}

uint32_t memoryGetKey (uint16_t index){
	uint32_t byteaddr = 0;
	if(index > memsLen) return 0;
	for (uint16_t i = 0; i < index; i++) {
		if(mems[i].type & MEM_GROUP) continue;
		byteaddr += memoryTypeSize(mems[i].type);
	}
	return byteaddr;
}

uint8_t memoryTypeSize(uint8_t type){
	return typeLength[type];
}

uint8_t memoryGroupSize(uint16_t index){
	if(index > memsLen) return 0;
	if(!(mems[index].type & MEM_GROUP) && (mems[index].type & MEM_START)) return 0;

	uint8_t bytesize = 0;
	while(1){
		index++;
		if(mems[index].type & MEM_GROUP)break;
		bytesize += memoryTypeSize(mems[index].type);
	}
	return bytesize;
}

void memoryLogicInit(void){
	mems    = &_mem_start;
	memsLen = &_mem_stop - &_mem_start;

	/* Print the table */
	#ifdef MEM_DEBUG
	serialPrint("[+] MEM Length : %d\n", memsLen);

	char* groupname = mems[0].name;
	uint8_t bytesize;

	int addr = 0;
	for(uint16_t i = 0; i < memsLen; i++){
		if(mems[i].type & MEM_GROUP){
			if(mems[i].type & MEM_START){
				groupname = mems[i].name;
				bytesize = memoryGroupSize(i);
			}
			else continue;
			serialPrint("[%d] %s.%s : %d bytes\n",addr,groupname,mems[i].name, bytesize);
		}
		else{
			bytesize = memoryTypeSize(mems[i].type);
			addr = memoryGetKey(i);
			serialPrint("[%d] %s.%s : %d bytes\n",addr,groupname,mems[i].name, bytesize);
		}

	}
	#endif
}
