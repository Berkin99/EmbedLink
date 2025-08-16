
#ifndef UAVEXE_H_
#define UAVEXE_H_

#include <stdint.h>

/**
 * 
 */

typedef enum{
    UAVEXE_PARSE,
    UAVEXE_SET,
    UAVEXE_DELAY,
}uavexe_e;

void uavexeInit(void);
void uavexeTask(void* argv);
void uavexeParse(uint8_t* data);

void uavexeSet(uint8_t* data);
void uavexeLaunch(uint8_t* data);


#endif