/**
 * @file    app_config.h
 * @brief   Central application constants and default configuration.
 *          Change defaults and magic numbers here for easier tuning.
 */
#ifndef __APP_CONFIG_H
#define __APP_CONFIG_H

/* ---------------------------------------------------------------------------*
 * NOx / O2 sensor conversion defaults (slope a, intercept b; value = a*raw+b)
 * ---------------------------------------------------------------------------*/
#define DEFAULT_NOX_A    0.05f
#define DEFAULT_NOX_B   -200.0f
#define DEFAULT_O2_A     0.000514f
#define DEFAULT_O2_B   -12.0f

/* 3-point calibration: default Y values when not stored in Flash */
#define NOX_CALIBRATION_NUM   3
#define NOX_Y0    0.0f
#define NOX_Y1    1500.0f
#define NOX_Y2    2500.0f

#define O2_CALIBRATION_NUM   3
#define O2_Y0     0.0f
#define O2_Y1     12.5f
#define O2_Y2     25.0f

/* Default alarm thresholds: NOx high (ppm), O2 low (%) */
#define DEFAULT_NOX_HIGH_ALARM   2000.0f
#define DEFAULT_O2_LOW_ALARM    15.0f

/* Blowback defaults: duration (s), interval (s). Interval must be > duration. */
#define DEFAULT_BLOW_DURATION   60u
#define DEFAULT_BLOW_INTERVAL   3600u
/** Minimum blowback duration (s); P25 below this is clamped to avoid timer period 0. */
#define BLOW_DURATION_MIN_S     1u

/* 4–20 mA output: full-scale NOx (ppm) and O2 (%) */
#define NOX_FS_PPM   2500.0f
#define O2_FS_PCT    25.0f
/* 4 mA = 4000, 20 mA = 20000 in internal units (e.g. 0.1 µA or DAC step) */
#define MA4_BASE     4000u
#define MA20_RANGE   16000u

/* J1939 / CAN */
#define J1939_HEATER_CAN_ID    0x18FEDF55u
#define J1939_HEATER_PAYLOAD_TAIL  0x55u

/* Multi-sensor: max channels (for future 3-way extension) */
#define NOX_SENSOR_COUNT_MAX   3u
/* Current number of sensors (2: SA 0x52 outlet, SA 0x51 inlet) */
#define NOX_SENSOR_COUNT       2u
/* Source addresses per channel: [0]=0x52, [1]=0x51 */
#define NOX_SENSOR_SA_LIST     { 0x52u, 0x51u }

/* Work mode: which channel(s) drive the single 4-20mA output */
typedef enum {
    NOX_MODE_SINGLE = 0,       /* Only channel 0 (SA 0x52); backward compatible */
    NOX_MODE_PRIMARY_BACKUP,   /* Use first valid channel; switch on fault/blowback */
    NOX_MODE_FUSION           /* Average of all valid channels */
} NoxWorkMode_t;

#endif /* __APP_CONFIG_H */
