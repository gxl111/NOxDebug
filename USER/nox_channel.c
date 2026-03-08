/**
 * @file    nox_channel.c
 * @brief   Per-sensor channel: init, update from CAN, validity.
 *          Params per channel for future 3-way; currently 2 channels (SA 0x52, 0x51).
 */
#include "nox_channel.h"
#include "app_config.h"
#include <string.h>

/* Source addresses: [0]=outlet 0x52, [1]=inlet 0x51 (see Electrical Interface doc). */
static const uint8_t s_sa_list[] = { 0x52u, 0x51u };

static NoxWorkMode_t s_work_mode = NOX_MODE_SINGLE;

NoxChannel_t g_noxChannels[NOX_SENSOR_COUNT_MAX];

void NoxChannel_Init(void)
{
    memset(g_noxChannels, 0, sizeof(g_noxChannels));
    for (uint8_t ch = 0; ch < NOX_SENSOR_COUNT; ch++) {
        NoxChannel_t *c = &g_noxChannels[ch];
        c->source_address = s_sa_list[ch];
        NoxSensor_SetDefaultNOx(&c->nox_low);
        NoxSensor_SetDefaultNOx(&c->nox_high);
        NoxSensor_SetDefaultO2(&c->o2_low);
        NoxSensor_SetDefaultO2(&c->o2_high);
        c->nox_y[0] = NOX_Y0; c->nox_y[1] = NOX_Y1; c->nox_y[2] = NOX_Y2;
        c->o2_y[0]  = O2_Y0;  c->o2_y[1]  = O2_Y1;  c->o2_y[2]  = O2_Y2;
        NoxSensor_CalibrationInit(c->nox_x, c->nox_y, &c->nox_low, &c->nox_high);
        NoxSensor_CalibrationInit(c->o2_x,  c->o2_y,  &c->o2_low,  &c->o2_high);
    }
}

/* Build 9-bit state word from J1939 status/heater/FMI bytes (same as original NOx_Handle). */
static uint16_t build_state(uint8_t statusByte, uint8_t heaterByte, uint8_t errNOx, uint8_t errO2)
{
    uint16_t state = (1u << 8);
    uint8_t voltageInRange = statusByte & 0x03u;
    uint8_t sensorAtTemp   = (statusByte >> 2) & 0x03u;
    uint8_t NOxStable      = (statusByte >> 4) & 0x03u;
    uint8_t O2Stable       = (statusByte >> 6) & 0x03u;
    uint8_t heaterControl   = (heaterByte >> 5) & 0x03u;
    uint8_t errHeater      = heaterByte & 0x1Fu;

    if (NOxStable == 0x01u) state |= (1u << 7);
    if (O2Stable  == 0x01u) state |= (1u << 6);
    if (sensorAtTemp == 0x01u) state |= (1u << 5);
    if (voltageInRange == 0x01u) state |= (1u << 4);
    state |= (1u << 3);
    if (heaterControl == 0x00u || heaterControl == 0x01u || heaterControl == 0x02u)
        ; /* heating */
    else
        state &= (uint16_t)~((uint16_t)(1u << 3));
    if (errHeater == 0x1Fu) state |= (1u << 2);
    if (errNOx   == 0x1Fu) state |= (1u << 1);
    if (errO2    == 0x1Fu) state |= (1u << 0);
    return state;
}

void NoxChannel_UpdateFromCan(uint8_t ch_index, const uint8_t *data)
{
    if (ch_index >= NOX_SENSOR_COUNT || !data) return;

    NoxChannel_t *c = &g_noxChannels[ch_index];
    c->raw_nox = (uint16_t)((data[1] << 8) | data[0]);
    c->raw_o2  = (uint16_t)((data[3] << 8) | data[2]);

    c->nox_ppm = NoxSensor_RawToValue(c->raw_nox, c->nox_x, &c->nox_low, &c->nox_high);
    c->o2_pct  = NoxSensor_RawToValue(c->raw_o2,  c->o2_x,  &c->o2_low,  &c->o2_high);

    uint8_t statusByte = data[4];
    uint8_t heaterByte = data[5];
    uint8_t errNOx     = data[6] & 0x1Fu;
    uint8_t errO2      = data[7] & 0x1Fu;

    c->state = build_state(statusByte, heaterByte, errNOx, errO2);
    /* Valid when all FMIs 0x1F and NOx/O2 stable and at temp (same as state 0x1FF). */
    c->valid = (c->state == 0x1FFu) ? 1u : 0u;
}

uint8_t NoxChannel_IsValid(uint8_t ch_index)
{
    if (ch_index >= NOX_SENSOR_COUNT) return 0;
    return g_noxChannels[ch_index].valid;
}

void NoxChannel_SetWorkMode(NoxWorkMode_t mode)
{
    s_work_mode = mode;
}

void NoxChannel_GetCurrentOutput(float *nox_ppm, float *o2_pct, uint16_t *state)
{
    float n = 0.0f, o = 0.0f;
    uint16_t s = 0u;
    uint8_t valid_count = 0u;

    if (s_work_mode == NOX_MODE_SINGLE) {
        /* Only channel 0 (backward compatible). */
        NoxChannel_t *c = &g_noxChannels[0];
        n = c->nox_ppm;
        o = c->o2_pct;
        s = c->state;
        if (nox_ppm) *nox_ppm = n;
        if (o2_pct) *o2_pct = o;
        if (state) *state = s;
        return;
    }

    if (s_work_mode == NOX_MODE_PRIMARY_BACKUP) {
        /* Use first valid channel (0 then 1). */
        for (uint8_t ch = 0; ch < NOX_SENSOR_COUNT; ch++) {
            if (NoxChannel_IsValid(ch)) {
                NoxChannel_t *c = &g_noxChannels[ch];
                if (nox_ppm) *nox_ppm = c->nox_ppm;
                if (o2_pct) *o2_pct = c->o2_pct;
                if (state) *state = c->state;
                return;
            }
        }
        /* No valid channel: keep last values from channel 0. */
        NoxChannel_t *c = &g_noxChannels[0];
        if (nox_ppm) *nox_ppm = c->nox_ppm;
        if (o2_pct) *o2_pct = c->o2_pct;
        if (state) *state = c->state;
        return;
    }

    if (s_work_mode == NOX_MODE_FUSION) {
        /* Average of all valid channels. */
        for (uint8_t ch = 0; ch < NOX_SENSOR_COUNT; ch++) {
            if (NoxChannel_IsValid(ch)) {
                NoxChannel_t *c = &g_noxChannels[ch];
                n += c->nox_ppm;
                o += c->o2_pct;
                s = c->state; /* last valid state for status display */
                valid_count++;
            }
        }
        if (valid_count > 0u) {
            n /= (float)valid_count;
            o /= (float)valid_count;
        }
        if (nox_ppm) *nox_ppm = n;
        if (o2_pct) *o2_pct = o;
        if (state) *state = s;
        return;
    }

    if (nox_ppm) *nox_ppm = n;
    if (o2_pct) *o2_pct = o;
    if (state) *state = s;
}
