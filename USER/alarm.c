/**
 * @file    alarm.c
 * @brief   Alarm logic: Modbus P12/P13 (NOx high, O₂ low) and comparison with live values.
 */
#include "alarm.h"
#include "modbus_slave.h"
#include "nox_sensor.h"

void Alarm_Update(void)
{
    float nox_high, o2_low;
    Var_Read_AlarmCfg(&nox_high, &o2_low);
    NOx_High = nox_high;
    O2_Low   = o2_low;
    if (NOx_ppm > NOx_High) { /* NOx high alarm – extend here (e.g. relay, message) */ }
    if (O2_pct < O2_Low)     { /* O2 low alarm – extend here */ }
}
