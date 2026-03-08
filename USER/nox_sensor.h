/**
 * @file    nox_sensor.h
 * @brief   NOx/O2 sensor calibration, conversion (raw -> ppm/%), and 4–20 mA output.
 *          Keeps sensor math and parameters in one place for readability and reuse.
 */
#ifndef __NOX_SENSOR_H
#define __NOX_SENSOR_H

#include <stdint.h>
#include "app_config.h"

/* Linear conversion parameter: value = a * raw + b */
typedef struct {
    float a;
    float b;
} NoxSensorParam_t;
/* Alias for compatibility with NOx.c and nox_sensor.c */
typedef NoxSensorParam_t Parameter;

/* Calibration point arrays: X = raw ADC/count, Y = physical value (ppm or %) */
#define NOX_CAL_NUM   NOX_CALIBRATION_NUM
#define O2_CAL_NUM    O2_CALIBRATION_NUM

/* Default parameters (first and second segment) */
extern NoxSensorParam_t NOx_parameter;
extern NoxSensorParam_t NOx_parameter1;
extern NoxSensorParam_t O2_parameter;
extern NoxSensorParam_t O2_parameter1;

/* Calibration points: X (raw), Y (ppm or %) */
extern uint16_t NOx_x[NOX_CAL_NUM];
extern float    NOx_y[NOX_CAL_NUM];
extern uint16_t O2_x[O2_CAL_NUM];
extern float    O2_y[O2_CAL_NUM];

/* Raw and converted values (updated by NOx_Handle) */
extern uint16_t NOx_raw;
extern uint16_t O2_raw;
extern float    NOx_ppm;
extern float    O2_pct;

/* Alarm thresholds (read from Modbus; used by Alarm()) */
extern float NOx_High;
extern float O2_Low;

/**
 * Set NOx parameter to default slope/intercept.
 */
void NoxSensor_SetDefaultNOx(NoxSensorParam_t *p);

/**
 * Set O2 parameter to default slope/intercept.
 */
void NoxSensor_SetDefaultO2(NoxSensorParam_t *p);

/**
 * Initialize calibration X from current Y and segment parameters (inverse linear).
 * Call after loading params from Flash or after reset-to-default.
 */
void NoxSensor_CalibrationInit(uint16_t *x, const float *y,
                               const NoxSensorParam_t *p0,
                               const NoxSensorParam_t *p1);

/**
 * Compute slope and intercept from two points (x[0],y[0]) and (x[1],y[1]).
 * @return 1 on success, 0 if x[1]==x[0].
 */
int NoxSensor_CalcSlopeIntercept(const uint16_t *x, const float *y, NoxSensorParam_t *p);

/**
 * Convert raw to physical value using two-segment linear calibration.
 * Segment 1: raw <= x[1]; segment 2: raw > x[1].
 */
float NoxSensor_RawToValue(uint16_t raw,
                           const uint16_t *x,
                           const NoxSensorParam_t *p_low,
                           const NoxSensorParam_t *p_high);

/**
 * Convert NOx (ppm) and O2 (%) to 4–20 mA representation.
 * Output: 4 bytes, big-endian [NOx_hi, NOx_lo, O2_hi, O2_lo].
 */
void NoxSensor_To4_20mA(float nox_ppm, float o2_pct, uint8_t *buf);

#endif /* __NOX_SENSOR_H */
