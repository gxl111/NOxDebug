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
/** Stagger between ch0 and ch1 periodic blowback start (s). Ch1 fires at (n*interval + stagger). */
#define BLOW_STAGGER_SEC        300u

/** No J1939 NOx frame on this channel for this long => valid=0, state bit9 link-lost (see README 5.4). */
#define NOX_CAN_SILENCE_TIMEOUT_MS   2000u

/* 4-20 mA output: full-scale NOx (ppm) and O2 (%) */
#define NOX_FS_PPM   2500.0f
#define O2_FS_PCT    25.0f
/* 4 mA = 4000, 20 mA = 20000 in internal units (e.g. 0.1 ?A or DAC step) */
#define MA4_BASE     4000u
#define MA20_RANGE   16000u

/* J1939 / CAN */
#define J1939_HEATER_CAN_ID    0x18FEDF55u
#define J1939_HEATER_PAYLOAD_TAIL  0x55u
/* 1=使能 MCP2515 第二路 CAN；0=禁用 */
#define NOX_USE_MCP2515        1
/* Keep the MCP2515 path short-fail so CAN2 issues cannot stall CAN1/OLED updates. */
#define MCP2515_SPI_TIMEOUT_MS          2u
#define MCP2515_TX_BUSY_TIMEOUT_MS      3u
#define MCP2515_TX_COMPLETE_TIMEOUT_MS  5u
#define NOX_CAN1_RX_DRAIN_LIMIT         8u
#define NOX_MCP2515_RX_DRAIN_LIMIT      4u
#define NOX_MCP2515_HEATER_PERIOD_MS    250u
#define NOX_MCP2515_HEATER_RETRY_MS     1000u
#define NOX_RECEIVE_QUEUE_WAIT_MS       10u
#define NOX_RECEIVE_LOOP_DELAY_MS       20u

/* Multi-sensor: max channels (for future 3-way extension) */
#define NOX_SENSOR_COUNT_MAX   3u
/* Current number of sensors: ch0=CAN1 SA 0x52, ch1=CAN1 SA 0x51, ch2=CAN2(MCP2515) SA 0x52 */
#define NOX_SENSOR_COUNT       3u
/* Source addresses per channel: [0]=0x52 outlet, [1]=0x51 inlet, [2]=0x52 second CAN */
#define NOX_SENSOR_SA_LIST     { 0x52u, 0x51u, 0x52u }

/*
 * Factory Flash programming on boot:
 * 1 = After Register_Init, run FactoryFlash_ProgramDefaults() then do not Load; RAM keeps FF until next boot.
 *     Set back to 0 and rebuild for normal operation (Load on boot) or every boot will overwrite Flash.
 * 0 = Normal: LoadRegistersFromFlash() on boot.
 */
#define FACTORY_FLASH_PROGRAM_ON_BOOT   0

/* Work mode: which channel(s) drive the single 4-20mA output */
typedef enum {
    NOX_MODE_SINGLE = 0,       /* Only channel 0 (SA 0x52); backward compatible */
    NOX_MODE_PRIMARY_BACKUP,   /* Use first valid channel; switch on fault/blowback */
    NOX_MODE_FUSION           /* Average of all valid channels */
} NoxWorkMode_t;

/*
 * Sensor power control: 0 = do not drive GPIO (power_on register still R/W in Modbus).
 * 1 = drive reserved GPIO from each sensor's power_on register (see main.h SENSOR_POWERx).
 * GPIO pins reserved in main.h; change when hardware is defined.
 */
#define SENSOR_POWER_GPIO_ENABLE   0

/*
 * Relay hardware test mode:
 * 1 = after GPIO init, skip normal app and continuously test J1-J9 and JC1-JC3:
 *     all off -> each relay on/off in order -> all on -> all off.
 * 0 = normal firmware.
 */
#define RELAY_TEST_MODE       0
#define RELAY_TEST_STEP_MS    700u
#define RELAY_TEST_GAP_MS     300u

/* Debounce automatic suction relay changes caused by transient sensor status flips. */
#define SUCTION_VALVE_DEBOUNCE_MS   1000u

/*
 * OLED（I2C1 PB6/PB7）：1 = 编译并周期性刷新 S1/S2/S3、输出与运行时间；0 = 不引用 oled 驱动。
 */
#define APP_USE_OLED   1

#endif /* __APP_CONFIG_H */
