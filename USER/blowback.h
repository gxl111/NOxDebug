/**
 * @file    blowback.h
 * @brief   Blowback: two valves. D01/D02 sensor1 normal/blowback, D03/D04 sensor2 normal/blowback.
 *          Only one sensor may blow at a time (no simultaneous blowback, including in fusion mode).
 */
#ifndef __BLOWBACK_H
#define __BLOWBACK_H

#include <stdint.h>
#include "app_config.h"

/* Turn blowback valve on (1) or off (0). ch=0: Relay0/1, ch=1: Relay2/3. */
void BLOW_CONTROL(uint8_t ch, uint8_t state);

/** 1 if channel is currently in blowback (use other channel for output when dual-sensor). */
uint8_t Blowback_IsChannelBlowing(uint8_t ch);

/* Run each cycle from NOxDefault: update both valves from P24-P28 and P29-P33. */
void Blowback_Update(void);

/* Called once from NOxDefault before the main loop; creates timers for both channels. */
void Blowback_Init(void);

/* Get/set config for channel 0 (P24/P25). Used by Register_Init and AfterFlash_Init. */
uint32_t Blowback_GetInterval(void);
uint32_t Blowback_GetDuration(void);
void Blowback_SetConfig(uint32_t interval_s, uint32_t duration_s);

/* Channel 1 config (P29/P30). */
uint32_t Blowback_GetIntervalCh1(void);
uint32_t Blowback_GetDurationCh1(void);
void Blowback_SetConfigCh1(uint32_t interval_s, uint32_t duration_s);

#endif /* __BLOWBACK_H */
