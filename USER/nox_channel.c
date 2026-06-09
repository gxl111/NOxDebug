/**
 * @file    nox_channel.c
 * @brief   Per-sensor channel: init, update from CAN, validity.
 *          Params per channel for S1/S2/S3 (CAN1 SA 0x52/0x51, CAN2 SA 0x52).
 *          Runtime strategy: blowback overrides; fusion only when both valid and not blowing;
 *          otherwise degrades to primary-backup; both invalid => fault readback (high 0xFF).
 *          某路反吹和反吹后抽气恢复延时内：该路 raw/ppm/state/valid 冻结为切阀前最后值
 *          （仍刷新 last_rx_ms，且不做静默超时改写）。
 */
#include "nox_channel.h"
#include "app_config.h"
#include "blowback.h"
#include "main.h"
#include <string.h>

/** Bit 9: no J1939 NOx message within NOX_CAN_SILENCE_TIMEOUT_MS (disconnect / bus fault). */
#define NOX_STATE_BIT_LINK_LOST  (1u << 9)

/** 尚未收到 CAN 或静默超时：与 UpdateCommTimeouts 超时分支一致（bit8+bit9，valid=0） */
static uint16_t NoxChannel_StateWordLinkLost(void)
{
    return (uint16_t)((1u << 8) | NOX_STATE_BIT_LINK_LOST);
}

/*
 * Active channel after GetCurrentOutput: 0=S1, 1=S2, 2=fusion average applied.
 * Readback high byte for Modbus: 0=S1, 1=S2, 2=fusion, 0xFF=fault (both invalid).
 */
static uint8_t s_active_output_channel = 0u;
static uint8_t s_readback_high_byte = 0u; /* 0,1,2, or NOX_READBACK_FAULT */

#define NOX_READBACK_FAULT  0xFFu

uint8_t NoxChannel_GetActiveOutputChannel(void)
{
    return s_active_output_channel;
}

uint8_t NoxChannel_GetWorkModeReadbackHighByte(void)
{
    return s_readback_high_byte;
}

/* P35 output sensor: 0b01=S1/ch0, 0b10=S2/ch1, 0b11=fusion, 0b100=S3/ch2(CAN2), 0b00=fault */
#define OUT_REG_S0      0x01u
#define OUT_REG_S1      0x02u
#define OUT_REG_FUSION  0x03u
#define OUT_REG_S2      0x04u
#define OUT_REG_FAULT   0x00u

uint8_t NoxChannel_GetOutputSensorReg(void)
{
    if (s_readback_high_byte == NOX_READBACK_FAULT)
        return OUT_REG_FAULT;
    if (s_readback_high_byte == 3u)
        return OUT_REG_FUSION;
    if (s_readback_high_byte == 2u)
        return OUT_REG_S2;
    if (s_readback_high_byte == 1u)
        return OUT_REG_S1;
    return OUT_REG_S0;
}

static uint8_t channel_selectable(uint8_t ch)
{
    return (!Blowback_IsChannelDataHold(ch) && NoxChannel_IsValid(ch)) ? 1u : 0u;
}

/* When a channel is in blowback/data hold, output must use another settled path. */
static void copy_channel_out(uint8_t ch, float *nox_ppm, float *o2_pct, uint16_t *state)
{
    if (ch >= NOX_SENSOR_COUNT) ch = 0u;
    s_active_output_channel = ch;
    s_readback_high_byte = ch; /* 0 or 1 */
    NoxChannel_t *c = &g_noxChannels[ch];
    if (nox_ppm) *nox_ppm = c->nox_ppm;
    if (o2_pct) *o2_pct = c->o2_pct;
    if (state) *state = c->state;
}

/* Output averaged fusion (all valid channels); readback high = 2 */
static void copy_fusion_out(float *nox_ppm, float *o2_pct, uint16_t *state)
{
    float n = 0.0f, o = 0.0f;
    uint8_t n_valid = 0u;
    for (uint8_t ch = 0; ch < NOX_SENSOR_COUNT; ch++) {
        if (!channel_selectable(ch)) continue;
        NoxChannel_t *c = &g_noxChannels[ch];
        n += c->nox_ppm;
        o += c->o2_pct;
        n_valid++;
    }
    if (n_valid > 0u) {
        n /= (float)n_valid;
        o /= (float)n_valid;
    }
    s_active_output_channel = 2u;
    s_readback_high_byte = 3u;  /* 3 = 融合（与 ch2 的 2 区分） */
    if (nox_ppm) *nox_ppm = n;
    if (o2_pct) *o2_pct = o;
    if (state) *state = (n_valid > 0u) ? 0x1FFu : 0u;
}

/* Both channels invalid: fill from fallback_ch for continuity; readback marks fault */
static void copy_fault_fallback(float *nox_ppm, float *o2_pct, uint16_t *state, uint8_t fallback_ch)
{
    if (fallback_ch >= NOX_SENSOR_COUNT) fallback_ch = 0u;
    s_active_output_channel = fallback_ch;
    s_readback_high_byte = NOX_READBACK_FAULT;
    NoxChannel_t *c = &g_noxChannels[fallback_ch];
    if (nox_ppm) *nox_ppm = c->nox_ppm;
    if (o2_pct) *o2_pct = c->o2_pct;
    if (state) *state = c->state;
}

/*
 * Primary-backup resolution (also used when fusion cannot average).
 * first_channel: preferred channel index (0, 1, or 2).
 * 1) Exactly one channel in blowback/data hold -> use first valid settled channel
 * 2) Try first_channel, then others in order
 * 3) No valid -> fault fallback (primary channel data)
 */
static void resolve_primary_backup_ex(float *nox_ppm, float *o2_pct, uint16_t *state, uint8_t first_channel)
{
    uint8_t n_hold = 0u;
    for (uint8_t ch = 0; ch < NOX_SENSOR_COUNT; ch++)
        if (Blowback_IsChannelDataHold(ch)) n_hold++;
    if (n_hold == 1u) {
        for (uint8_t ch = 0; ch < NOX_SENSOR_COUNT; ch++) {
            if (channel_selectable(ch)) {
                copy_channel_out(ch, nox_ppm, o2_pct, state);
                return;
            }
        }
    }
    if (channel_selectable(first_channel)) {
        copy_channel_out(first_channel, nox_ppm, o2_pct, state);
        return;
    }
    for (uint8_t ch = 0; ch < NOX_SENSOR_COUNT; ch++) {
        if (ch == first_channel) continue;
        if (channel_selectable(ch)) {
            copy_channel_out(ch, nox_ppm, o2_pct, state);
            return;
        }
    }
    copy_fault_fallback(nox_ppm, o2_pct, state, first_channel);
}

/* Source addresses per channel (see Electrical Interface Gen 2.8): from NOX_SENSOR_SA_LIST */
static const uint8_t s_sa_list[] = NOX_SENSOR_SA_LIST;

static NoxWorkMode_t s_work_mode = NOX_MODE_PRIMARY_BACKUP;
static uint8_t s_single_channel_index = 0u; /* mode 0: single ch; mode 1: 主 ch */

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
        c->valid = 0u;
        c->state = NoxChannel_StateWordLinkLost();
        /* last_rx_ms 保持 0：表示尚未收到过本通道 CAN，bit9 链路丢失直至首帧 */
    }
}

/*
 * Status Byte = CAN payload byte 4, data[4] (Electrical Interface Gen 2.8, table 4.1.1).
 * Encoding is 2-bit fields; interpret Bit1+Bit0, Bit3+Bit2, etc. together閳ユ敃ingle bits have no meaning alone.
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
 * Ref: APN_SNS_02_020 tables 4.1.1, 4.1.2, 4.1.3a閳ユ徆.
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
    /* 数据保持期仍刷新接收时刻，避免静默超时覆盖切阀前锁存的测量 */
    c->last_rx_ms = HAL_GetTick();
    if (Blowback_IsChannelDataHold(ch_index))
        return;

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

void NoxChannel_UpdateCommTimeouts(uint32_t now_ms)
{
    for (uint8_t ch = 0; ch < NOX_SENSOR_COUNT; ch++) {
        NoxChannel_t *c = &g_noxChannels[ch];
        if (Blowback_IsChannelDataHold(ch))
            continue;
        uint32_t last = c->last_rx_ms;
        if (last == 0u) {
            /* 上电后尚未收到任何本通道帧：立即保持链路丢失（bit9=1），不等待静默超时 */
            c->valid = 0u;
            c->state = NoxChannel_StateWordLinkLost();
            continue;
        }
        if ((now_ms - last) >= (uint32_t)NOX_CAN_SILENCE_TIMEOUT_MS) {
            c->valid = 0u;
            c->state = NoxChannel_StateWordLinkLost();
        }
    }
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
    s_single_channel_index = (ch_index < NOX_SENSOR_COUNT) ? ch_index : 0u;
}

void NoxChannel_GetCurrentOutput(float *nox_ppm, float *o2_pct, uint16_t *state)
{
    if (s_work_mode == NOX_MODE_SINGLE) {
        uint8_t ch = (s_single_channel_index < NOX_SENSOR_COUNT) ? s_single_channel_index : 0u;
        if (Blowback_IsChannelDataHold(ch)) {
            for (uint8_t k = 0; k < NOX_SENSOR_COUNT; k++) {
                if (k == ch) continue;
                if (channel_selectable(k)) {
                    copy_channel_out(k, nox_ppm, o2_pct, state);
                    return;
                }
            }
        }
        if (!channel_selectable(ch)) {
            for (uint8_t k = 0; k < NOX_SENSOR_COUNT; k++) {
                if (k == ch) continue;
                if (channel_selectable(k)) {
                    copy_channel_out(k, nox_ppm, o2_pct, state);
                    return;
                }
            }
            copy_fault_fallback(nox_ppm, o2_pct, state, ch);
        } else {
            copy_channel_out(ch, nox_ppm, o2_pct, state);
        }
        return;
    }

    if (s_work_mode == NOX_MODE_PRIMARY_BACKUP) {
        /* 主从：优先 主 (s_single_channel_index)，再从 */
        resolve_primary_backup_ex(nox_ppm, o2_pct, state, s_single_channel_index);
        return;
    }

    if (s_work_mode == NOX_MODE_FUSION) {
        /* 融合：多路有效取平均，否则按主备选一路 */
        uint8_t n_ok = 0u;
        for (uint8_t ch = 0; ch < NOX_SENSOR_COUNT; ch++) {
            if (channel_selectable(ch)) n_ok++;
        }
        if (n_ok >= 2u) {
            copy_fusion_out(nox_ppm, o2_pct, state);
            return;
        }
        resolve_primary_backup_ex(nox_ppm, o2_pct, state, 0u);
        return;
    }

    if (nox_ppm) *nox_ppm = 0.0f;
    if (o2_pct) *o2_pct = 0.0f;
    if (state) *state = 0u;
}
