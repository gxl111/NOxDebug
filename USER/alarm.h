/**
 * @file    alarm.h
 * @brief   Alarm thresholds: sync P12/P13 with NOx_High/O2_Low and compare to current NOx/O₂ output.
 */
#ifndef __ALARM_H
#define __ALARM_H

/* Run each cycle from NOxDefault: read P12/P13, update NOx_High/O2_Low, compare to NOx_ppm and O₂ output (O2_pct). */
void Alarm_Update(void);

#endif /* __ALARM_H */
