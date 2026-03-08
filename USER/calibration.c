/**
 * @file    calibration.c
 * @brief   NOx/O2 3-point calibration per channel. Each sensor has its own cal trigger/point select (no P53).
 *          Updates g_noxChannels[ch] and g_tVar S1/S2.
 */
#include "calibration.h"
#include "modbus_slave.h"
#include "nox_sensor.h"
#include "nox_channel.h"
#include "app_config.h"

static uint8_t CalcSlopeIntercept(uint16_t *x, float *y, Parameter *p)
{
    if (!x || !y || !p || x[1] == x[0]) return 0;
    p->a = (y[1] - y[0]) / (float)(x[1] - x[0]);
    p->b = y[0] - p->a * (float)x[0];
    return 1;
}

/* Sync channel params to g_tVar S1 or S2. */
static void SyncChannelToVar(uint8_t ch)
{
    NoxChannel_t *c = &g_noxChannels[ch];
    Var_Write_SensorSeg1NoxA(ch, c->nox_low.a);
    Var_Write_SensorSeg1NoxB(ch, c->nox_low.b);
    Var_Write_SensorSeg1O2A(ch, c->o2_low.a);
    Var_Write_SensorSeg1O2B(ch, c->o2_low.b);
    Var_Write_SensorSeg2NoxA(ch, c->nox_high.a);
    Var_Write_SensorSeg2NoxB(ch, c->nox_high.b);
    Var_Write_SensorSeg2O2A(ch, c->o2_high.a);
    Var_Write_SensorSeg2O2B(ch, c->o2_high.b);
    Var_Write_SensorP2Nox(ch, c->nox_y[1]);
    Var_Write_SensorP2O2(ch, c->o2_y[1]);
    Var_Write_SensorP3Nox(ch, c->nox_y[2]);
    Var_Write_SensorP3O2(ch, c->o2_y[2]);
}

void Calibration_NOx(Parameter *p, Parameter *p1)
{
    for (uint8_t ch = 0; ch < NOX_SENSOR_COUNT; ch++) {
        NoxChannel_t *c = &g_noxChannels[ch];
        p->a = c->nox_low.a;  p->b = c->nox_low.b;
        p1->a = c->nox_high.a; p1->b = c->nox_high.b;

        uint16_t cmd = Var_Read_SensorNoxCalTrig(ch);
        if (cmd == 0x0001u) {
            float y1 = Var_Read_SensorP2Nox(ch);
            float y2 = Var_Read_SensorP3Nox(ch);
            switch (Var_Read_SensorNoxPtSel(ch)) {
                case 0:
                    c->nox_x[0] = c->raw_nox;
                    CalcSlopeIntercept(c->nox_x, c->nox_y, &c->nox_low);
                    break;
                case 1:
                    c->nox_y[1] = y1;
                    c->nox_x[1] = c->raw_nox;
                    CalcSlopeIntercept(c->nox_x, c->nox_y, &c->nox_low);
                    CalcSlopeIntercept(c->nox_x + 1, c->nox_y + 1, &c->nox_high);
                    break;
                case 2:
                    c->nox_y[2] = y2;
                    c->nox_x[2] = c->raw_nox;
                    CalcSlopeIntercept(c->nox_x + 1, c->nox_y + 1, &c->nox_high);
                    break;
                default:
                    Var_Write_SensorNoxCalTrig(ch, 0x0005u);
                    return;
            }
            Var_Write_SensorNoxCalTrig(ch, 0x000Fu);
            SyncChannelToVar(ch);
            InternalFlash_Write();
        } else if (cmd == 0x0002u) {
            NoxSensor_SetDefaultNOx(&c->nox_low);
            NoxSensor_SetDefaultNOx(&c->nox_high);
            Var_Write_SensorNoxCalTrig(ch, 0x0010u);
            NoxSensor_CalibrationInit(c->nox_x, c->nox_y, &c->nox_low, &c->nox_high);
            SyncChannelToVar(ch);
            InternalFlash_Write();
        }
    }
}

void Calibration_O2(Parameter *p, Parameter *p1)
{
    for (uint8_t ch = 0; ch < NOX_SENSOR_COUNT; ch++) {
        NoxChannel_t *c = &g_noxChannels[ch];
        p->a = c->o2_low.a;   p->b = c->o2_low.b;
        p1->a = c->o2_high.a; p1->b = c->o2_high.b;

        uint16_t cmd = Var_Read_SensorO2CalTrig(ch);
        if (cmd == 0x0001u) {
            float y1 = Var_Read_SensorP2O2(ch);
            float y2 = Var_Read_SensorP3O2(ch);
            switch (Var_Read_SensorO2PtSel(ch)) {
                case 0:
                    c->o2_x[0] = c->raw_o2;
                    CalcSlopeIntercept(c->o2_x, c->o2_y, &c->o2_low);
                    break;
                case 1:
                    c->o2_y[1] = y1;
                    c->o2_x[1] = c->raw_o2;
                    CalcSlopeIntercept(c->o2_x, c->o2_y, &c->o2_low);
                    CalcSlopeIntercept(c->o2_x + 1, c->o2_y + 1, &c->o2_high);
                    break;
                case 2:
                    c->o2_y[2] = y2;
                    c->o2_x[2] = c->raw_o2;
                    CalcSlopeIntercept(c->o2_x + 1, c->o2_y + 1, &c->o2_high);
                    break;
                default:
                    Var_Write_SensorO2CalTrig(ch, 0x0005u);
                    return;
            }
            Var_Write_SensorO2CalTrig(ch, 0x000Fu);
            SyncChannelToVar(ch);
            InternalFlash_Write();
        } else if (cmd == 0x0002u) {
            NoxSensor_SetDefaultO2(&c->o2_low);
            NoxSensor_SetDefaultO2(&c->o2_high);
            Var_Write_SensorO2CalTrig(ch, 0x0010u);
            NoxSensor_CalibrationInit(c->o2_x, c->o2_y, &c->o2_low, &c->o2_high);
            SyncChannelToVar(ch);
            InternalFlash_Write();
        }
    }
}

void Calibration_Init(void)
{
    for (uint8_t ch = 0; ch < NOX_SENSOR_COUNT; ch++) {
        NoxChannel_t *c = &g_noxChannels[ch];
        NoxSensor_CalibrationInit(c->nox_x, c->nox_y, &c->nox_low, &c->nox_high);
        NoxSensor_CalibrationInit(c->o2_x, c->o2_y, &c->o2_low, &c->o2_high);
    }
}
