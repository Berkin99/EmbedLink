#ifndef MEM_H_
#define MEM_H_

struct mem_s{
	uint8_t type;
	char * name;
	void * address;
};

typedef struct mem_s mem_t;

#define MEM_BYTES_MASK 0x03
#define MEM_1BYTE  0x00
#define MEM_2BYTES 0x01
#define MEM_4BYTES 0x02
#define MEM_8BYTES 0x03

#define MEM_TYPE_MASK   0x0F
#define MEM_TYPE_INT   (0x00<<2)
#define MEM_TYPE_FLOAT (0x01<<2)

#define MEM_SIGNED 		(0x00<<3)
#define MEM_UNSIGNED 	(0x01<<3)

#define MEM_UINT8  (MEM_1BYTE  | MEM_TYPE_INT | MEM_UNSIGNED)
#define MEM_INT8   (MEM_1BYTE  | MEM_TYPE_INT | MEM_SIGNED)
#define MEM_UINT16 (MEM_2BYTES | MEM_TYPE_INT | MEM_UNSIGNED)
#define MEM_INT16  (MEM_2BYTES | MEM_TYPE_INT | MEM_SIGNED)
#define MEM_UINT32 (MEM_4BYTES | MEM_TYPE_INT | MEM_UNSIGNED)
#define MEM_INT32  (MEM_4BYTES | MEM_TYPE_INT | MEM_SIGNED)

#define MEM_FLOAT  (MEM_4BYTES | MEM_TYPE_FLOAT | MEM_SIGNED)

#define MEM_CORE   (1<<5)
#define MEM_RONLY  (1<<6)

#define MEM_START 1
#define MEM_STOP  0

#define MEM_VARIABLE (0x00<<7)
#define MEM_GROUP    (0x01<<7)

#define MEM_PERSISTENT (1<<8)

#define MEM_ADD(TYPE, NAME, ADDRESS) \
    { .type = (TYPE),\
      .name = #NAME, \
      .address = (void*)(ADDRESS) },

#define MEM_GROUP_START(NAME)  \
  static struct mem_s __mems_##NAME[] __attribute__((section(".mem."#NAME), used)) = { \
  MEM_ADD(MEM_GROUP | MEM_START, NAME, 0x0) \

#define MEM_GROUP_STOP(NAME) \
  MEM_ADD(MEM_GROUP | MEM_STOP, stop_##NAME, 0x0) \
};

#endif /* MEM_H_ */
