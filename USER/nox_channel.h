/**
 * @file    nox_channel.h
 * @brief   Per-sensor channel data and params (modular for 2 or 3 sensors).
 *          Each channel has its own raw/ppm/state and calibration params.
 */
#ifndef __NOX_CHANNEL_H
#define __NOX_CHANNEL_H

#include <stdint.h>
#include "app_config.h"
#include "nox_sensor.h"

/* Single channel state and conversion params */
typedef struct {
    uint8_t  source_address;   /* J1939 SA: 0x52 or 0x51 */
    uint8_t  valid;             /* 1 = data valid (NOx/O2 stable, at temp, no FMI) */
    uint16_t raw_nox;
    uint16_t raw_o2;
    float    nox_ppm;
    float    o2_pct;
    uint16_t state;             /* status: J1939-derived bits 0..8 + bit9 link-lost (see README) */
    uint32_t last_rx_ms;        /* HAL_GetTick() at last NoxChannel_UpdateFromCan; 0 = never */
    /* Calibration: segment params */
    NoxSensorParam_t nox_low, nox_high;
    NoxSensorParam_t o2_low,  o2_high;
    /* Calibration points: X (raw), Y (ppm or %) */
    uint16_t nox_x[NOX_CAL_NUM];
    float    nox_y[NOX_CAL_NUM];
    uint16_t o2_x[O2_CAL_NUM];
    float    o2_y[O2_CAL_NUM];
} NoxChannel_t;

/* Channel array: ch0=CAN1 SA 0x52, ch1=CAN1 SA 0x51, ch2=CAN2 SA 0x52. */
extern NoxChannel_t g_noxChannels[NOX_SENSOR_COUNT_MAX];

/** Initialize all channels: default params and SA from NOX_SENSOR_SA_LIST. */
void NoxChannel_Init(void);

/**
 * Update channel from one J1939 8-byte payload (NOx_raw, O2_raw, status, heater, FMI).
 * Fills raw, ppm, pct, state, valid for the given channel.
 *
 * CAN payload layout (table 4.1.1):
 *   data[0..1] NOx_raw, data[2..3] O2_raw,
 *   data[4]    Status Byte — four 2-bit fields (not single-bit flags):
 *              Bit1..0 power in range, Bit3..2 sensor at temp,
 *              Bit5..4 NOx stable, Bit7..6 O2 stable; 01 = valid, see nox_channel.c block comment.
 *   data[5]    Heater Byte, data[6] Error NOx, data[7] Error O2 (FMI in low 5 bits).
 */
void NoxChannel_UpdateFromCan(uint8_t ch_index, const uint8_t *data);

/** Call each control cycle (e.g. NOx_Receive) with HAL_GetTick(): marks timeout channels link-lost. */
void NoxChannel_UpdateCommTimeouts(uint32_t now_ms);

/** Return 1 if channel has valid measurement (for primary/backup or fusion). */
uint8_t NoxChannel_IsValid(uint8_t ch_index);

/** Set work mode (called from NOx.c after reading P34). */
void NoxChannel_SetWorkMode(NoxWorkMode_t mode);

/** Set selected channel index. Mode 0 = target channel; mode 1 = primary channel. */
void NoxChannel_SetSingleChannelIndex(uint8_t ch_index);

/**
 * Compute current output from all channels according to current work mode.
 * Single: selected target, with fallback on invalid/blowing. Primary_backup: primary then other valid paths.
 * Fusion: average of valid non-blowing channels when at least two are available.
 * Writes to *nox_ppm, *o2_pct (used as O2 in Modbus P02/sensor blocks), *state for P01/P02/P07 and 4-20mA.
 */
void NoxChannel_GetCurrentOutput(float *nox_ppm, float *o2_pct, uint16_t *state);

/**
 * Channel currently driving the combined output (updated each strategy cycle).
 * Legacy active channel indicator. Prefer P35/NoxChannel_GetOutputSensorReg() for 3-channel output source.
 */
uint8_t NoxChannel_GetActiveOutputChannel(void);

/**
 * High byte for P34 readback: 0=S1, 1=S2, 2=fusion, 0xFF=fault (no valid path).
 */
uint8_t NoxChannel_GetWorkModeReadbackHighByte(void);

/**
 * Output sensor register (P35, read-only): 0b01=S1/ch0, 0b10=S2/ch1,
 * 0b11=fusion, 0b100=S3/ch2, 0b00=fault.
 */
uint8_t NoxChannel_GetOutputSensorReg(void);

#endif /* __NOX_CHANNEL_H */
