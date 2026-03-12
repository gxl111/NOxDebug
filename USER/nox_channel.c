/**
 * @file    nox_channel.c
 * @brief   Per-sensor channel: init, update from CAN, validity.
 *          Params per channel for future 3-way; currently 2 channels (SA 0x52, 0x51).
 */
#include "nox_channel.h"
#include "app_config.h"
#include "blowback.h"
#include <string.h>

/* Last channel that actually drives P01/P02/P07 (0=S1/SA0x52, 1=S2/SA0x51; 2=fusion both). */
static uint8_t s_active_output_channel = 0u;

uint8_t NoxChannel_GetActiveOutputChannel(void)
{
    return s_active_output_channel;
}

/* When a channel is in blowback, output must use the other path (gas path is purge). */
static void copy_channel_out(uint8_t ch, float *nox_ppm, float *o2_pct, uint16_t *state)
{
    if (ch >= NOX_SENSOR_COUNT) ch = 0u;
    s_active_output_channel = ch;
    NoxChannel_t *c = &g_noxChannels[ch];
    if (nox_ppm) *nox_ppm = c->nox_ppm;
    if (o2_pct) *o2_pct = c->o2_pct;
    if (state) *state = c->state;
}

/* Source addresses: [0]=outlet 0x52, [1]=inlet 0x51 (see Electrical Interface doc). */
static const uint8_t s_sa_list[] = { 0x52u, 0x51u };

static NoxWorkMode_t s_work_mode = NOX_MODE_PRIMARY_BACKUP;
static uint8_t s_single_channel_index = 0u;

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

/*
 * Status Byte = CAN payload byte 4, data[4] (Electrical Interface Gen 2.8, table 4.1.1).
 * Encoding is 2-bit fields; interpret Bit1+Bit0, Bit3+Bit2, etc. together—single bits have no meaning alone.
 *
 *   Bit7..6  O2 stable (table 4.1.3d)
 *   Bit5..4  NOx stable (table 4.1.3c)
 *   Bit3..2  Sensor at operating temperature (table 4.1.3b)
 *   Bit1..0  Power in range (table 4.1.3a)
 *
 * Each 2-bit field (MSB..LSB of the pair):
 *   00 = condition false / invalid / not at temp / out of range
 *   01 = condition true / valid / at temp / in range
 *   10 = not used
 *   11 = not allowed (power-on default; after dew-point message may transition to 00)
 * Ref: APN_SNS_02_020 tables 4.1.1, 4.1.2, 4.1.3a–d.
 */
/* Build 9-bit state word from J1939 status/heater/FMI bytes (same as original NOx_Handle). */
static uint16_t build_state(uint8_t statusByte, uint8_t heaterByte, uint8_t errNOx, uint8_t errO2)
{
    uint16_t state = (1u << 8);
    /* Bits 1..0: power in range; valid = 01b */
    uint8_t voltageInRange = statusByte & 0x03u;
    /* Bits 3..2: sensor at temp; valid = 01b */
    uint8_t sensorAtTemp   = (statusByte >> 2) & 0x03u;
    /* Bits 5..4: NOx stable; valid = 01b */
    uint8_t NOxStable      = (statusByte >> 4) & 0x03u;
    /* Bits 7..6: O2 stable; valid = 01b */
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

    /* Byte 4 = Status Byte (2-bit fields, see comment block above) */
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

void NoxChannel_SetSingleChannelIndex(uint8_t ch_index)
{
    s_single_channel_index = (ch_index != 0u) ? 1u : 0u;
}

void NoxChannel_GetCurrentOutput(float *nox_ppm, float *o2_pct, uint16_t *state)
{
    float n = 0.0f, o = 0.0f;
    uint16_t s = 0u;
    uint8_t valid_count = 0u;

    if (s_work_mode == NOX_MODE_SINGLE) {
        /* Use selected channel; if that channel is blowing, use the other (dual-sensor). */
        uint8_t ch = (s_single_channel_index < NOX_SENSOR_COUNT) ? s_single_channel_index : 0u;
        if (NOX_SENSOR_COUNT >= 2u && Blowback_IsChannelBlowing(ch)) {
            uint8_t other = 1u - ch;
            if (!Blowback_IsChannelBlowing(other)) {
                copy_channel_out(other, nox_ppm, o2_pct, state);
                return;
            }
        }
        copy_channel_out(ch, nox_ppm, o2_pct, state);
        return;
    }

    if (s_work_mode == NOX_MODE_PRIMARY_BACKUP) {
        /* If one path is in blowback, always use the other (not blowing) path. */
        if (NOX_SENSOR_COUNT >= 2u) {
            if (Blowback_IsChannelBlowing(0u) && !Blowback_IsChannelBlowing(1u)) {
                copy_channel_out(1u, nox_ppm, o2_pct, state);
                return;
            }
            if (Blowback_IsChannelBlowing(1u) && !Blowback_IsChannelBlowing(0u)) {
                copy_channel_out(0u, nox_ppm, o2_pct, state);
                return;
            }
        }
        /* Neither blowing (or both blowing): use first valid channel 0 then 1. */
        for (uint8_t ch = 0; ch < NOX_SENSOR_COUNT; ch++) {
            if (NoxChannel_IsValid(ch)) {
                copy_channel_out(ch, nox_ppm, o2_pct, state);
                return;
            }
        }
        /* No valid channel: keep last values from channel 0. */
        copy_channel_out(0u, nox_ppm, o2_pct, state);
        return;
    }

    if (s_work_mode == NOX_MODE_FUSION) {
        /* Average valid channels that are not in blowback; if only one path usable, use it. */
        uint8_t last_valid_ch = 0u;
        for (uint8_t ch = 0; ch < NOX_SENSOR_COUNT; ch++) {
            if (Blowback_IsChannelBlowing(ch))
                continue;
            if (NoxChannel_IsValid(ch)) {
                NoxChannel_t *c = &g_noxChannels[ch];
                n += c->nox_ppm;
                o += c->o2_pct;
                s = c->state;
                valid_count++;
                last_valid_ch = ch;
            }
        }
        if (valid_count > 0u) {
            n /= (float)valid_count;
            o /= (float)valid_count;
            /* 2 = both averaged; 1 = only one valid channel contributing */
            s_active_output_channel = (valid_count >= 2u) ? 2u : last_valid_ch;
        } else if (NOX_SENSOR_COUNT >= 2u) {
            /* Only one path not blowing but invalid: still use it for output (purge path wrong gas). */
            if (!Blowback_IsChannelBlowing(0u) && Blowback_IsChannelBlowing(1u))
                copy_channel_out(0u, nox_ppm, o2_pct, state);
            else if (!Blowback_IsChannelBlowing(1u) && Blowback_IsChannelBlowing(0u))
                copy_channel_out(1u, nox_ppm, o2_pct, state);
            return;
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
