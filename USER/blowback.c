/**
 * @file    blowback.c
 * @brief   Blowback: two valves (sensor1 P24-P28, sensor2 P29-P33).
 *          D01=normal1, D02=blow1, D03=normal2, D04=blow2. Only one sensor may blow at a time (no simultaneous blowback).
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

/* Per-channel state (ch0 and ch1). Both use time_1s_blow for periodic schedule. */
typedef struct {
    TimerHandle_t timer;
    uint8_t blow_flag;
    volatile uint8_t blow_end_pending;
    uint8_t blow_was_stopped;
    uint32_t blow_start_time_1s;
    uint32_t blowtime;
    uint32_t blowspan;
} BlowCh_t;

static BlowCh_t s_ch[NOX_SENSOR_COUNT];

uint8_t Blowback_IsChannelBlowing(uint8_t ch)
{
    if (ch >= NOX_SENSOR_COUNT) return 0u;
    return s_ch[ch].blow_flag ? 1u : 0u;
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
            if (k != ch && s_ch[k].blow_flag)
                return;  /* Another channel is blowing, skip starting this one */
        }
        sc->blow_flag = 1;
        sc->blow_start_time_1s = time_1s;
        xTimerChangePeriod(sc->timer, pdMS_TO_TICKS((uint32_t)sc->blowtime * 1000u), portMAX_DELAY);
        xTimerStart(sc->timer, portMAX_DELAY);
    } else {
        sc->blow_flag = 0;
    }
    /* 阀门保持寄存器驱动 J1-J9；反吹时仅控制反吹阀 J2(ch0)/J5(ch1)/J8(ch2) */
    if (ch == 0) {
        HAL_GPIO_WritePin(J2_IN_GPIO_Port, J2_IN_Pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);   /* S1 反吹 */
    } else if (ch == 1) {
        HAL_GPIO_WritePin(J5_IN_GPIO_Port, J5_IN_Pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);   /* S2 反吹 */
    } else {
        HAL_GPIO_WritePin(J8_IN_GPIO_Port, J8_IN_Pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);   /* S3 反吹 */
    }
    if (!state)
        Modbus_ApplySensorValveBlowToGPIO(ch);   /* 反吹结束，将 valve_blow 寄存器写回 GPIO */
}

uint32_t Blowback_GetInterval(void) { return s_ch[0].blowspan; }
uint32_t Blowback_GetDuration(void)  { return s_ch[0].blowtime; }
void Blowback_SetConfig(uint32_t interval_s, uint32_t duration_s)
{
    s_ch[0].blowspan = interval_s ? interval_s : DEFAULT_BLOW_INTERVAL;
    s_ch[0].blowtime = duration_s ? duration_s : DEFAULT_BLOW_DURATION;
    if (s_ch[0].blowtime < BLOW_DURATION_MIN_S)
        s_ch[0].blowtime = BLOW_DURATION_MIN_S;
    if (s_ch[0].blowspan > 0u && s_ch[0].blowtime >= s_ch[0].blowspan)
        s_ch[0].blowtime = s_ch[0].blowspan - 1u;
}

uint32_t Blowback_GetIntervalCh1(void) { return s_ch[1].blowspan; }
uint32_t Blowback_GetDurationCh1(void)  { return s_ch[1].blowtime; }
void Blowback_SetConfigCh1(uint32_t interval_s, uint32_t duration_s)
{
    s_ch[1].blowspan = interval_s ? interval_s : DEFAULT_BLOW_INTERVAL;
    s_ch[1].blowtime = duration_s ? duration_s : DEFAULT_BLOW_DURATION;
    if (s_ch[1].blowtime < BLOW_DURATION_MIN_S)
        s_ch[1].blowtime = BLOW_DURATION_MIN_S;
    if (s_ch[1].blowspan > 0u && s_ch[1].blowtime >= s_ch[1].blowspan)
        s_ch[1].blowtime = s_ch[1].blowspan - 1u;
}

uint32_t Blowback_GetIntervalCh2(void) { return s_ch[2].blowspan; }
uint32_t Blowback_GetDurationCh2(void)  { return s_ch[2].blowtime; }
void Blowback_SetConfigCh2(uint32_t interval_s, uint32_t duration_s)
{
    s_ch[2].blowspan = interval_s ? interval_s : DEFAULT_BLOW_INTERVAL;
    s_ch[2].blowtime = duration_s ? duration_s : DEFAULT_BLOW_DURATION;
    if (s_ch[2].blowtime < BLOW_DURATION_MIN_S)
        s_ch[2].blowtime = BLOW_DURATION_MIN_S;
    if (s_ch[2].blowspan > 0u && s_ch[2].blowtime >= s_ch[2].blowspan)
        s_ch[2].blowtime = s_ch[2].blowspan - 1u;
}

void Blowback_Init(void)
{
    for (uint8_t ch = 0; ch < NOX_SENSOR_COUNT; ch++) {
        BlowCh_t *sc = &s_ch[ch];
        if (sc->timer == NULL) {
            sc->blowtime = DEFAULT_BLOW_DURATION;
            sc->blowspan = DEFAULT_BLOW_INTERVAL;
            sc->timer = xTimerCreate("blowtimer", pdMS_TO_TICKS((uint32_t)sc->blowtime * 1000u), pdFALSE, (void *)(uint32_t)ch, Blow_Time_Out_Cb);
            if (sc->timer == NULL)
                Error_Handler();
        }
    }
}

static void Blowback_UpdateOne(uint8_t ch)
{
    uint16_t interval = Var_Read_SensorBlowInterval(ch);
    uint16_t duration = Var_Read_SensorBlowDuration(ch);
    uint16_t cmd = Var_Read_SensorBlowCmd(ch);

    BlowCh_t *sc = &s_ch[ch];

    if (sc->blow_end_pending) {
        sc->blow_end_pending = 0;
        LOCK_VAR();
        if (ch == 0) g_tVar.S1.blow_interval = (uint16_t)sc->blowspan;
        else if (ch == 1) g_tVar.S2.blow_interval = (uint16_t)sc->blowspan;
        else g_tVar.S3.blow_interval = (uint16_t)sc->blowspan;
        UNLOCK_VAR();
    }

    if (cmd == 3u || cmd == 0xFFFFu) {
        BLOW_CONTROL(ch, 0);
        xTimerStop(sc->timer, portMAX_DELAY);
        sc->blow_was_stopped = 1;
        Var_Write_SensorBlowCmd(ch, 0);
        cmd = 0;
    }
    if (cmd == 1u) {
        if (!sc->blow_flag)
            BLOW_CONTROL(ch, 1);
        Var_Write_SensorBlowCmd(ch, 0);
        cmd = 0;
    }
    if (cmd == 2u) {
        BLOW_CONTROL(ch, 1);
        time_1s_blow = 0;
        sc->blow_was_stopped = 0;
        Var_Write_SensorBlowCmd(ch, 0);
        cmd = 0;
    }

    if (interval == 0xFFFFu || interval == 0u) {
        BLOW_CONTROL(ch, 0);
        xTimerStop(sc->timer, portMAX_DELAY);
        sc->blow_was_stopped = 1;
        sc->blowspan = 0u;
    } else {
        uint32_t new_span = (uint32_t)interval;
        if (new_span != sc->blowspan) {
            sc->blowspan = new_span;
            time_1s_blow = 0;
            sc->blow_was_stopped = 0;
        }
        if (sc->blow_was_stopped) {
            time_1s_blow = 0;
            sc->blow_was_stopped = 0;
        }
    }

    sc->blowtime = (uint32_t)duration;
    if (sc->blowtime < BLOW_DURATION_MIN_S)
        sc->blowtime = BLOW_DURATION_MIN_S;
    if (sc->blowspan > 0u && sc->blowtime >= sc->blowspan)
        sc->blowtime = sc->blowspan - 1u;

    uint32_t tick = time_1s_blow;
    if (sc->blowspan != 0u && tick != 0u && !sc->blow_flag) {
        /* Ch0: fire at 0. Ch1: fire at stagger. Ch2: fire at span*2/3 (no relay, timer/reg only). */
        uint8_t fire = 0u;
        if (ch == 0u) {
            if (tick % sc->blowspan == 0u)
                fire = 1u;
        } else if (ch == 1u) {
            uint32_t phase = BLOW_STAGGER_SEC % sc->blowspan;
            if (phase == 0u)
                phase = sc->blowspan / 2u ? sc->blowspan / 2u : 1u;
            if ((tick % sc->blowspan) == phase)
                fire = 1u;
        } else {
            uint32_t phase = (sc->blowspan * 2u) / 3u;
            if (phase == 0u) phase = 1u;
            if ((tick % sc->blowspan) == phase)
                fire = 1u;
        }
        if (fire)
            BLOW_CONTROL(ch, 1);
    }

    /*
     * blow_countdown (40049 / 40087), always in seconds:
     * - blow_status == 0 (idle): interval countdown = seconds until next periodic blow start.
     * - blow_status == 1 (blowing): duration countdown = seconds remaining until blow ends.
     */
    {
        uint32_t cd = 0u;
        if (sc->blow_flag) {
            /* Blowing: show remaining blow duration */
            uint32_t elapsed = time_1s - sc->blow_start_time_1s;
            if (elapsed < sc->blowtime)
                cd = sc->blowtime - elapsed;
            else
                cd = 0u; /* timer callback will clear valve shortly */
        } else if (sc->blowspan != 0u) {
            /* Idle: show seconds until next scheduled blow (interval phase) */
            uint32_t span = sc->blowspan;
            uint32_t r = tick % span;
            if (ch == 0u) {
                cd = (r == 0u) ? span : (span - r);
            } else if (ch == 1u) {
                uint32_t phase = BLOW_STAGGER_SEC % span;
                if (phase == 0u) phase = span / 2u ? span / 2u : 1u;
                if (r < phase) cd = phase - r;
                else if (r > phase) cd = span - r + phase;
                else cd = span;
            } else {
                uint32_t phase = (span * 2u) / 3u;
                if (phase == 0u) phase = 1u;
                if (r < phase) cd = phase - r;
                else if (r > phase) cd = span - r + phase;
                else cd = span;
            }
        }
        LOCK_VAR();
        if (ch == 0) {
            g_tVar.S1.blow_status = sc->blow_flag ? 1u : 0u;
            g_tVar.S1.blow_countdown = (uint16_t)(cd > 65535u ? 65535u : cd);
        } else if (ch == 1) {
            g_tVar.S2.blow_status = sc->blow_flag ? 1u : 0u;
            g_tVar.S2.blow_countdown = (uint16_t)(cd > 65535u ? 65535u : cd);
        } else {
            g_tVar.S3.blow_status = sc->blow_flag ? 1u : 0u;
            g_tVar.S3.blow_countdown = (uint16_t)(cd > 65535u ? 65535u : cd);
        }
        UNLOCK_VAR();
    }
}

void Blowback_Update(void)
{
    Blowback_UpdateOne(0);
    Blowback_UpdateOne(1);
    Blowback_UpdateOne(2);
}
