



#include "sysconfig.h"
#include "systime.h"

#ifdef RF52_SPI

#include "telemetry_rf52.h"
#include "rtos.h"
#include "gpio.h"
#include "uart.h"
#include "spi.h"

taskAllocateStatic(RF52, TRX_TASK_STACK, TRX_TASK_PRI)
void _telemetryTaskRF52(void* argv);

int8_t telemetryInitRF52(void){
    pinWrite(RF52_CS, HIGH);
    taskCreateStatic(RF52, _telemetryTaskRF52, NULL);
    return OK;
}

int8_t telemetryTestRF52(void){
    return OK;
}

void _telemetryTaskRF52(void* argv){

    while (1){
        uint8_t txData = 0x36;
        uint8_t rxBuffer[2] = {0};
        pinWrite(RF52_CS, HIGH);
        int8_t rslt = spiTransmitReceive(&RF52_SPI, rxBuffer, &txData, 2);
        pinWrite(RF52_CS, HIGH);
        serialPrint("[>] RF52 Status : %d | Buffer : 0x%x : 0x%x \n", rslt, rxBuffer[0], rxBuffer[1]);
        delay(1400);
    }
}

int8_t telemetryReceiveRF52(uint8_t* pRxBuffer, uint16_t length){
    return 0;

}

int8_t telemetryTransmitRF52(const uint8_t* pTxData, uint16_t length){
    return 0;

}

int8_t telemetryIsReadyRF52(void){
    return 0;
}

void   telemetryWaitDataReadyRF52(void){

}

#endif