/**
 * @file    calibration.h
 * @brief   NOx/O2 calibration: P08/P10 commands, slope/intercept from points, sync with g_tVar and Flash.
 */
#ifndef __CALIBRATION_H
#define __CALIBRATION_H

#include "nox_sensor.h"

/* Run NOx calibration step from P08/P09; update params and Flash on success. */
void Calibration_NOx(Parameter *p, Parameter *p1);

/* Run O2 calibration step from P10/P11; update params and Flash on success. */
void Calibration_O2(Parameter *p, Parameter *p1);

/* Initialize calibration X arrays from current Y and segment params (after Flash load or reset). */
void Calibration_Init(void);

#endif /* __CALIBRATION_H */
