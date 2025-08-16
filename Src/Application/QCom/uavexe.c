

#include "uavexe.h"
#include "xqueue.h"
#include "executer.h"

void uavexeInit(void);
void uavexeTask(void* argv);
void uavexeParse(uint8_t* data);

void uavexeSet(uint8_t* data);
void uavexeLaunch(uint8_t* data);
