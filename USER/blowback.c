/**
 * @file    blowback.c
 * @brief   Blowback for S1/S2/S3 (J2/J5/J8 blow valves).
 *          Only one sensor may blow at a time.
 *          反吹时关抽气（正常）继电器、开反吹继电器；结束按寄存器恢复两路。
 */
#include "blowback.h"
#include "modbus_slave.h"
#include "app_config.h"
#include "main.h"
#include "gpio.h"
#include "FreeRTOS.h"
#include "timers.h"

extern uint32_t time_1s;
extern uint32_t time_1s_blow;

/* Per-channel state. All channels share time_1s_blow and one period. */
typedef struct {
    TimerHandle_t timer;
    uint8_t blow_flag;
    volatile uint8_t blow_end_pending;
    uint8_t blow_was_stopped;
    uint8_t blow_active_latched;
    uint32_t data_hold_until_ms;
    uint32_t blow_start_time_1s;
    uint32_t blowtime;
    uint32_t blowspan;
} BlowCh_t;

static BlowCh_t s_ch[NOX_SENSOR_COUNT];
static uint32_t s_shared_blowspan = DEFAULT_BLOW_INTERVAL;

static void Blowback_NormalizeTiming(BlowCh_t *sc);

static uint32_t Blowback_PhaseSec(uint8_t ch, uint32_t span)
{
    if (span == 0u)
        return 0u;
    return ((uint32_t)ch * span) / BLOW_PHASE_DIVISOR;
}

static uint32_t Blowback_FirstFireSec(uint8_t ch, uint32_t span)
{
    uint32_t phase = Blowback_PhaseSec(ch, span);
    return (phase == 0u) ? span : phase;
}

static void Blowback_WriteSharedIntervalReg(uint16_t interval)
{
    LOCK_VAR();
    g_tVar.S1.blow_interval = interval;
    g_tVar.S2.blow_interval = interval;
    g_tVar.S3.blow_interval = interval;
    UNLOCK_VAR();
}

static void Blowback_SetSharedInterval(uint32_t interval_s, uint8_t reset_cycle)
{
    uint32_t span = (interval_s == 0xFFFFu) ? 0u : interval_s;
    s_shared_blowspan = span;
    for (uint8_t ch = 0u; ch < NOX_SENSOR_COUNT; ch++) {
        s_ch[ch].blowspan = span;
        Blowback_NormalizeTiming(&s_ch[ch]);
    }
    Blowback_WriteSharedIntervalReg((span > 65535u) ? 65535u : (uint16_t)span);
    if (reset_cycle)
        time_1s_blow = 0u;
}

static void Blowback_SyncSharedIntervalFromRegs(void)
{
    uint16_t target = (uint16_t)(s_shared_blowspan > 65535u ? 65535u : s_shared_blowspan);
    uint16_t regs[NOX_SENSOR_COUNT] = {
        Var_Read_SensorBlowInterval(0),
        Var_Read_SensorBlowInterval(1),
        Var_Read_SensorBlowInterval(2)
    };

    for (uint8_t ch = 0u; ch < NOX_SENSOR_COUNT; ch++) {
        if (regs[ch] != target) {
            uint32_t next = (regs[ch] == 0xFFFFu) ? 0u : (uint32_t)regs[ch];
            Blowback_SetSharedInterval(next, 1u);
            return;
        }
    }
}

static void Blowback_NormalizeTiming(BlowCh_t *sc)
{
    if (sc->blowtime < BLOW_DURATION_MIN_S)
        sc->blowtime = BLOW_DURATION_MIN_S;
    if (sc->blowspan > BLOW_DURATION_MIN_S && sc->blowtime >= sc->blowspan)
        sc->blowtime = sc->blowspan - 1u;
}

uint8_t Blowback_IsChannelBlowing(uint8_t ch)
{
    if (ch >= NOX_SENSOR_COUNT) return 0u;
    if (s_ch[ch].blow_flag)
        return 1u;
    return (Var_Read_SensorValve(ch, 1u) != 0u) ? 1u : 0u;
}

static uint32_t Blowback_RecoveryDelayMs(void)
{
    return (uint32_t)BLOW_RECOVERY_DELAY_S * 1000u;
}

void Blowback_OnBlowValveChanged(uint8_t ch, uint8_t on)
{
    if (ch >= NOX_SENSOR_COUNT) return;
    BlowCh_t *sc = &s_ch[ch];

    if (on) {
        sc->blow_active_latched = 1u;
        sc->data_hold_until_ms = 0u;
        return;
    }

    if (sc->blow_active_latched) {
        sc->blow_active_latched = 0u;
        sc->data_hold_until_ms = HAL_GetTick() + Blowback_RecoveryDelayMs();
    }
}

uint8_t Blowback_IsChannelDataHold(uint8_t ch)
{
    if (ch >= NOX_SENSOR_COUNT) return 0u;
    if (Blowback_IsChannelBlowing(ch))
        return 1u;

    uint32_t until = s_ch[ch].data_hold_until_ms;
    if (until == 0u)
        return 0u;
    if ((int32_t)(HAL_GetTick() - until) < 0)
        return 1u;

    s_ch[ch].data_hold_until_ms = 0u;
    return 0u;
}

static void Blow_Time_Out_Cb(TimerHandle_t xTimer)
{
    uint8_t ch = (uint8_t)(uint32_t)pvTimerGetTimerID(xTimer);
    BLOW_CONTROL(ch, 0);
    s_ch[ch].blow_end_pending = 1;
}

void BLOW_CONTROL(uint8_t ch, uint8_t state)
{
    if (ch >= NOX_SENSOR_COUNT) return;
    BlowCh_t *sc = &s_ch[ch];
    /* Only one sensor may blow at a time (even in fusion mode). */
    if (state) {
        for (uint8_t k = 0; k < NOX_SENSOR_COUNT; k++) {
            if (k != ch && Blowback_IsChannelBlowing(k))
                return;  /* Another channel is blowing, skip starting this one */
        }
        sc->blow_flag = 1;
        sc->blow_start_time_1s = time_1s;
        xTimerChangePeriod(sc->timer, pdMS_TO_TICKS((uint32_t)sc->blowtime * 1000u), portMAX_DELAY);
        xTimerStart(sc->timer, portMAX_DELAY);
    } else {
        sc->blow_flag = 0;
    }
    /* Keep valve registers and GPIO in sync so Modbus/UI state matches relay output. */
    if (state) {
        Modbus_ClearManualSuctionOverride(ch);
        Var_Write_SensorValve(ch, 1u, 1u);
    } else {
        Var_Write_SensorValve(ch, 1u, 0u);
        Modbus_ApplySensorValveNormalToGPIO(ch);
    }
}

uint32_t Blowback_GetInterval(void) { return s_shared_blowspan; }
uint32_t Blowback_GetDuration(void)  { return s_ch[0].blowtime; }
void Blowback_SetConfig(uint32_t interval_s, uint32_t duration_s)
{
    Blowback_SetSharedInterval(interval_s ? interval_s : DEFAULT_BLOW_INTERVAL, 1u);
    s_ch[0].blowtime = duration_s ? duration_s : DEFAULT_BLOW_DURATION;
    Blowback_NormalizeTiming(&s_ch[0]);
}

uint32_t Blowback_GetIntervalCh1(void) { return s_shared_blowspan; }
uint32_t Blowback_GetDurationCh1(void)  { return s_ch[1].blowtime; }
void Blowback_SetConfigCh1(uint32_t interval_s, uint32_t duration_s)
{
    Blowback_SetSharedInterval(interval_s ? interval_s : DEFAULT_BLOW_INTERVAL, 1u);
    s_ch[1].blowtime = duration_s ? duration_s : DEFAULT_BLOW_DURATION;
    Blowback_NormalizeTiming(&s_ch[1]);
}

uint32_t Blowback_GetIntervalCh2(void) { return s_shared_blowspan; }
uint32_t Blowback_GetDurationCh2(void)  { return s_ch[2].blowtime; }
void Blowback_SetConfigCh2(uint32_t interval_s, uint32_t duration_s)
{
    Blowback_SetSharedInterval(interval_s ? interval_s : DEFAULT_BLOW_INTERVAL, 1u);
    s_ch[2].blowtime = duration_s ? duration_s : DEFAULT_BLOW_DURATION;
    Blowback_NormalizeTiming(&s_ch[2]);
}

void Blowback_Init(void)
{
    for (uint8_t ch = 0; ch < NOX_SENSOR_COUNT; ch++) {
        BlowCh_t *sc = &s_ch[ch];
        if (sc->timer == NULL) {
            sc->blowtime = DEFAULT_BLOW_DURATION;
            sc->blowspan = s_shared_blowspan;
            sc->timer = xTimerCreate("blowtimer", pdMS_TO_TICKS((uint32_t)sc->blowtime * 1000u), pdFALSE, (void *)(uint32_t)ch, Blow_Time_Out_Cb);
            if (sc->timer == NULL)
                Error_Handler();
        }
    }
}

static void Blowback_UpdateOne(uint8_t ch)
{
    uint16_t duration = Var_Read_SensorBlowDuration(ch);
    uint16_t cmd = Var_Read_SensorBlowCmd(ch);
    uint8_t cal_active = (Var_Read_SensorValve(ch, 2u) != 0u) ? 1u : 0u;

    BlowCh_t *sc = &s_ch[ch];

    if (sc->blow_end_pending) {
        sc->blow_end_pending = 0;
        LOCK_VAR();
        if (ch == 0) g_tVar.S1.blow_interval = (uint16_t)sc->blowspan;
        else if (ch == 1) g_tVar.S2.blow_interval = (uint16_t)sc->blowspan;
        else g_tVar.S3.blow_interval = (uint16_t)sc->blowspan;
        UNLOCK_VAR();
    }

    if (cal_active) {
        Modbus_ClearManualSuctionOverride(ch);
        BLOW_CONTROL(ch, 0);
        xTimerStop(sc->timer, portMAX_DELAY);
        sc->blow_was_stopped = 1;
        Var_Write_SensorBlowCmd(ch, 0);
        Var_Write_SensorValve(ch, 0u, 0u);
        Var_Write_SensorValve(ch, 1u, 0u);
        LOCK_VAR();
        if (ch == 0) {
            g_tVar.S1.blow_status = 0u;
            g_tVar.S1.blow_countdown = 0u;
        } else if (ch == 1) {
            g_tVar.S2.blow_status = 0u;
            g_tVar.S2.blow_countdown = 0u;
        } else {
            g_tVar.S3.blow_status = 0u;
            g_tVar.S3.blow_countdown = 0u;
        }
        UNLOCK_VAR();
        return;
    }

    if (cmd == 3u || cmd == 0xFFFFu) {
        BLOW_CONTROL(ch, 0);
        xTimerStop(sc->timer, portMAX_DELAY);
        sc->blow_was_stopped = 1;
        Var_Write_SensorBlowCmd(ch, 0);
        cmd = 0;
    }
    if (cmd == 1u) {
        if (!Blowback_IsChannelBlowing(ch))
            BLOW_CONTROL(ch, 1);
        Var_Write_SensorBlowCmd(ch, 0);
        cmd = 0;
    }
    if (cmd == 2u) {
        if (!Blowback_IsChannelBlowing(ch)) {
            BLOW_CONTROL(ch, 1);
            time_1s_blow = 0;
            sc->blow_was_stopped = 0;
        }
        Var_Write_SensorBlowCmd(ch, 0);
        cmd = 0;
    }

    if (sc->blowspan == 0u) {
        if (!sc->blow_flag) {
            BLOW_CONTROL(ch, 0);
            xTimerStop(sc->timer, portMAX_DELAY);
            sc->blow_was_stopped = 1;
        }
    } else if (sc->blow_was_stopped) {
        time_1s_blow = 0u;
        sc->blow_was_stopped = 0u;
    }

    sc->blowtime = (uint32_t)duration;
    Blowback_NormalizeTiming(sc);

    uint32_t tick = time_1s_blow;
    if (sc->blowspan != 0u && !Blowback_IsChannelBlowing(ch)) {
        uint32_t phase = Blowback_PhaseSec(ch, sc->blowspan);
        uint32_t first_fire = Blowback_FirstFireSec(ch, sc->blowspan);
        uint8_t fire = ((tick % sc->blowspan) == phase) ? 1u : 0u;
        if (tick < first_fire)
            fire = 0u;
        if (fire)
            BLOW_CONTROL(ch, 1);
    }

    /*
     * blow_countdown (S1 40051 / S2 40093 / S3 40135), always in seconds:
     * - blow_status == 0 (idle): interval countdown = seconds until next periodic blow start.
     * - blow_status == 1 (blowing): duration countdown = seconds remaining until blow ends.
     */
    {
        uint32_t cd = 0u;
        uint8_t blow_active = Blowback_IsChannelBlowing(ch);
        if (sc->blow_flag) {
            /* Blowing: show remaining blow duration */
            uint32_t elapsed = time_1s - sc->blow_start_time_1s;
            if (elapsed < sc->blowtime)
                cd = sc->blowtime - elapsed;
            else
                cd = 0u; /* timer callback will clear valve shortly */
        } else if (blow_active) {
            cd = 0u; /* manual blow valve is on; no timer countdown */
        } else if (sc->blowspan != 0u) {
            /* Idle: show seconds until next scheduled blow (interval phase) */
            uint32_t span = sc->blowspan;
            uint32_t phase = Blowback_PhaseSec(ch, span);
            uint32_t first_fire = Blowback_FirstFireSec(ch, span);
            if (tick < first_fire) {
                cd = first_fire - tick;
            } else {
                uint32_t r = tick % span;
                if (r < phase) cd = phase - r;
                else if (r > phase) cd = span - r + phase;
                else cd = span;
            }
        }
        LOCK_VAR();
        if (ch == 0) {
            g_tVar.S1.blow_status = blow_active ? 1u : 0u;
            g_tVar.S1.blow_countdown = (uint16_t)(cd > 65535u ? 65535u : cd);
        } else if (ch == 1) {
            g_tVar.S2.blow_status = blow_active ? 1u : 0u;
            g_tVar.S2.blow_countdown = (uint16_t)(cd > 65535u ? 65535u : cd);
        } else {
            g_tVar.S3.blow_status = blow_active ? 1u : 0u;
            g_tVar.S3.blow_countdown = (uint16_t)(cd > 65535u ? 65535u : cd);
        }
        UNLOCK_VAR();
    }
}

void Blowback_Update(void)
{
    Blowback_SyncSharedIntervalFromRegs();
    Blowback_UpdateOne(0);
    Blowback_UpdateOne(1);
    Blowback_UpdateOne(2);
}
