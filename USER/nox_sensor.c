/**
 * @file    nox_sensor.c
 * @brief   NOx/O2 calibration math and raw-to-value conversion.
 */
#include "nox_sensor.h"
#include <string.h>

Parameter NOx_parameter  = { DEFAULT_NOX_A,  DEFAULT_NOX_B };
Parameter NOx_parameter1 = { DEFAULT_NOX_A,  DEFAULT_NOX_B };
Parameter O2_parameter   = { DEFAULT_O2_A,   DEFAULT_O2_B };
Parameter O2_parameter1  = { DEFAULT_O2_A,   DEFAULT_O2_B };

uint16_t NOx_x[NOX_CAL_NUM];
float    NOx_y[NOX_CAL_NUM] = { NOX_Y0, NOX_Y1, NOX_Y2 };
uint16_t O2_x[O2_CAL_NUM];
float    O2_y[O2_CAL_NUM]   = { O2_Y0,  O2_Y1,  O2_Y2 };

uint16_t NOx_raw;
uint16_t O2_raw;
float    NOx_ppm;
float    O2_pct;

float NOx_High = DEFAULT_NOX_HIGH_ALARM;
float O2_Low   = DEFAULT_O2_LOW_ALARM;

void NoxSensor_SetDefaultNOx(Parameter *p)
{
    if (!p) return;
    p->a = DEFAULT_NOX_A;
    p->b = DEFAULT_NOX_B;
}

void NoxSensor_SetDefaultO2(Parameter *p)
{
    if (!p) return;
    p->a = DEFAULT_O2_A;
    p->b = DEFAULT_O2_B;
}

void NoxSensor_CalibrationInit(uint16_t *x, const float *y,
                               const Parameter *p0,
                               const Parameter *p1)
{
    if (!x || !y || !p0 || !p1) return;
    if (p0->a == 0.0f) return;
    x[0] = (uint16_t)((y[0] - p0->b) / p0->a);
    x[1] = (uint16_t)((y[1] - p0->b) / p0->a);
    if (p1->a == 0.0f) return;
    x[2] = (uint16_t)((y[2] - p1->b) / p1->a);
}

int NoxSensor_CalcSlopeIntercept(const uint16_t *x, const float *y, Parameter *p)
{
    if (!x || !y || !p) return 0;
    if (x[1] == x[0]) return 0;
    p->a = (y[1] - y[0]) / (float)(x[1] - x[0]);
    p->b = y[0] - p->a * (float)x[0];
    return 1;
}

float NoxSensor_RawToValue(uint16_t raw,
                           const uint16_t *x,
                           const Parameter *p_low,
                           const Parameter *p_high)
{
    if (raw <= x[1])
        return p_low->a * (float)raw + p_low->b;
    return p_high->a * (float)raw + p_high->b;
}

void NoxSensor_To4_20mA(float nox_ppm, float o2_pct, uint8_t *buf)
{
    uint16_t nox_u = MA4_BASE;
    uint16_t o2_u  = MA4_BASE;
    if (nox_ppm > 0.0f && NOX_FS_PPM > 0.0f)
        nox_u = (uint16_t)((nox_ppm * (float)MA20_RANGE) / NOX_FS_PPM + (float)MA4_BASE);
    if (o2_pct > 0.0f && O2_FS_PCT > 0.0f)
        o2_u  = (uint16_t)((o2_pct  * (float)MA20_RANGE) / O2_FS_PCT  + (float)MA4_BASE);

    if (buf) {
        buf[0] = (uint8_t)(nox_u >> 8);
        buf[1] = (uint8_t)(nox_u & 0xFF);
        buf[2] = (uint8_t)(o2_u >> 8);
        buf[3] = (uint8_t)(o2_u & 0xFF);
    }
}
