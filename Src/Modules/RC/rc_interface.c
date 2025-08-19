#include "rc_interface.h"
#include "xmath.h"
#include "systime.h"
#include "uart.h"

// Global RC handle
RC_Handle_t rc;

// Callbacks
static void (*eventCallback)(uint8_t event) = 0;

static void RC_ChannelAlign(RC_Channel_t* ch, uint8_t raw) {
    float val = (float)raw;
    
    if (ch->settings.max == 0) {
        ch->value = val;
        return;
    }
    
    val -= ch->settings.center;
    val = deadbandf32(val, ch->settings.deadband);
    
    if (val < 0) {
        val /= fabsf(ch->settings.min);
    } else if (val > 0) {
        val /= ch->settings.max;
    }
    
    ch->value = clampf32(val, -1.0f, 1.0f);
}

void RC_Init(void) {
    const RC_Channel_t midCfg = {0, {RC_CH_DEADBAND, RC_CH_CENTER, -127, 127}};
    const RC_Channel_t defCfg = {0, {0, 0, 0, 255}};
    const RC_Channel_t zeroCfg = {0, {0, 0, 0, 0}};
    
    rc.state = RC_DISARMED;
    rc.lastUpdate = 0;
    rc.ch[RC_CH_X] = midCfg;
    rc.ch[RC_CH_Y] = midCfg;
    rc.ch[RC_CH_Z] = midCfg;
    rc.ch[RC_CH_POWER] = defCfg;
    rc.ch[RC_CH_CONF] = zeroCfg;
}

void RC_SetCallback(void (*callback)(uint8_t event)){
    eventCallback = callback;
}

void RC_Update(uint8_t raw[5]) {
    for (uint8_t i = 0; i < RC_CH_LENGTH; ++i) RC_ChannelAlign(&rc.ch[i], raw[i]);

    rc.lastUpdate = millis();
    
    //serialPrint("[RC] %.2f, %.2f, %.2f, %.2f, %.2f [%d]\n ", rc.chPOWER.value, rc.chX.value, rc.chY.value, rc.chZ.value, rc.chCONF.value, rc.state);

    if (rc.chPOWER.value > 0.05f) return;
    
    if (rc.state == RC_ARMED) {
        if (rc.chPOWER.value == 0 && rc.chZ.value < -0.9f) {
            rc.state = RC_DISARMED;
            if (eventCallback) eventCallback(RC_EVENT_DISARM);
        }
    } else {
        if (rc.chPOWER.value == 0 && rc.chZ.value > 0.9f) {
            rc.state = RC_SWITCH;
        }
        if (rc.state == RC_SWITCH && rc.chZ.value < 0.1f) {
            rc.state = RC_ARMED;
            if (eventCallback) eventCallback(RC_EVENT_ARM);
        }
    }
}

void RC_Validity(void) {
    
    if (millis() - rc.lastUpdate > RC_CONNECTION_LOST_TIME_MS) {
        if (rc.state == RC_ARMED && eventCallback) {
            eventCallback(RC_EVENT_DISARM);
        }
        rc.state = RC_DISARMED;
    }
}