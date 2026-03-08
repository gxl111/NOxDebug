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
    uint16_t state;             /* 9-bit status same as P07 */
    /* Calibration: segment params */
    NoxSensorParam_t nox_low, nox_high;
    NoxSensorParam_t o2_low,  o2_high;
    /* Calibration points: X (raw), Y (ppm or %) */
    uint16_t nox_x[NOX_CAL_NUM];
    float    nox_y[NOX_CAL_NUM];
    uint16_t o2_x[O2_CAL_NUM];
    float    o2_y[O2_CAL_NUM];
} NoxChannel_t;

/* Channel array (index 0 = SA 0x52, index 1 = SA 0x51; extendable to 3) */
extern NoxChannel_t g_noxChannels[NOX_SENSOR_COUNT_MAX];

/** Initialize all channels: default params and SA from NOX_SENSOR_SA_LIST. */
void NoxChannel_Init(void);

/**
 * Update channel from one J1939 8-byte payload (NOx_raw, O2_raw, status, heater, FMI).
 * Fills raw, ppm, pct, state, valid for the given channel.
 */
void NoxChannel_UpdateFromCan(uint8_t ch_index, const uint8_t *data);

/** Return 1 if channel has valid measurement (for primary/backup or fusion). */
uint8_t NoxChannel_IsValid(uint8_t ch_index);

/** Set work mode (called from NOx.c after reading P34). */
void NoxChannel_SetWorkMode(NoxWorkMode_t mode);

/**
 * Compute current output from all channels according to current work mode.
 * Single: ch0 only; Primary_backup: first valid; Fusion: average of valid.
 * Writes to *nox_ppm, *o2_pct (used as O₂ in Modbus P02/sensor blocks), *state for P01/P02/P07 and 4-20mA.
 */
void NoxChannel_GetCurrentOutput(float *nox_ppm, float *o2_pct, uint16_t *state);

#endif /* __NOX_CHANNEL_H */
