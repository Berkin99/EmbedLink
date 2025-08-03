#ifndef RC_INTERFACE_H_
#define RC_INTERFACE_H_

#include <stdint.h>

#define RC_CONNECTION_LOST_TIME_MS 600
#define RC_CH_DEADBAND 10
#define RC_CH_CENTER 127
#define RC_CH_MAX 255
#define RC_CH_MIN 0
#define RC_CH_X 0
#define RC_CH_Y 1
#define RC_CH_Z 2
#define RC_CH_POWER 3
#define RC_CH_CONF 4
#define RC_CH_LENGTH 5

#define RC_EVENT_ARM 1
#define RC_EVENT_DISARM 2

typedef enum{
    RC_DISARMED = 0,
    RC_SWITCH,
    RC_ARMED,
}RC_State_e;

typedef struct{
    float value;
    struct {
        uint8_t deadband;
        uint8_t center;
        float min;
        float max;
    } settings;
}RC_Channel_t;

typedef struct {
    RC_State_e state;
    uint32_t lastUpdate;
    union{
        struct{
            RC_Channel_t chX;
            RC_Channel_t chY;
            RC_Channel_t chZ;
            RC_Channel_t chPOWER;
            RC_Channel_t chCONF;
        };
        RC_Channel_t ch[RC_CH_LENGTH];
    };
}RC_Handle_t;

// Global RC handle
extern RC_Handle_t rc;

// Functions
void RC_Init(void);
void RC_SetCallback(void (*callback)(uint8_t event));
void RC_Update(uint8_t raw[5]);
void RC_Validity(void);

#endif