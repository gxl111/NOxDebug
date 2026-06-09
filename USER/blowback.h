/**
 * @file    blowback.h
 * @brief   Blowback for S1/S2/S3. Each channel owns normal/blow/cal valve registers.
 *          Only one sensor may blow at a time (no simultaneous blowback, including in fusion mode).
 */
#ifndef __BLOWBACK_H
#define __BLOWBACK_H

#include <stdint.h>
#include "app_config.h"

/* Turn blowback valve on (1) or off (0). ch=0/1/2 => J2/J5/J8. */
void BLOW_CONTROL(uint8_t ch, uint8_t state);

/** 1 if channel is currently in blowback (use other channel for output when dual-sensor). */
uint8_t Blowback_IsChannelBlowing(uint8_t ch);

/**
 * 1 while channel data must be held: blow valve is active or the post-blowback
 * suction recovery delay has not elapsed yet. Output selection should skip it,
 * but suction-valve interlock should still use Blowback_IsChannelBlowing().
 */
uint8_t Blowback_IsChannelDataHold(uint8_t ch);

/* Notify blow valve transitions from direct Modbus valve writes. */
void Blowback_OnBlowValveChanged(uint8_t ch, uint8_t on);

/* Run each cycle from NOxDefault: update each channel's blowback command, timer and valves. */
void Blowback_Update(void);

/* Called once from NOxDefault before the main loop; creates timers for both channels. */
void Blowback_Init(void);

/* Get/set config for channel 0. Used by Register_Init and AfterFlash_Init. */
uint32_t Blowback_GetInterval(void);
uint32_t Blowback_GetDuration(void);
void Blowback_SetConfig(uint32_t interval_s, uint32_t duration_s);

/* Channel 1 config (P29/P30). */
uint32_t Blowback_GetIntervalCh1(void);
uint32_t Blowback_GetDurationCh1(void);
void Blowback_SetConfigCh1(uint32_t interval_s, uint32_t duration_s);

/* Channel 2 config (S3, second CAN, J8 blow valve). */
uint32_t Blowback_GetIntervalCh2(void);
uint32_t Blowback_GetDurationCh2(void);
void Blowback_SetConfigCh2(uint32_t interval_s, uint32_t duration_s);

#endif /* __BLOWBACK_H */
