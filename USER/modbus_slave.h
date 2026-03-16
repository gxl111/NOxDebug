/*
 * modbus_slave.h - Modbus RTU slave (holding registers + coils).
 * Common block: NOx/O2 outputs, 4-20mA mode, output values.
 * Per-sensor block: same layout as common registers, dual channels.
 * Flash save/load: see modbus_flash.h.
 */
#ifndef __MODBUS_SLAVE_H
#define __MODBUS_SLAVE_H
#include <stdint.h>
#include "FreeRTOS.h"
#include "semphr.h"
#include "gpio.h"
#include "usart.h"
#include "modbus.h"
#include "modbus_flash.h"

extern uint8_t SADDR485;
extern uint32_t SBAUD485;

extern SemaphoreHandle_t g_hVarMutex;

#define LOCK_VAR()    xSemaphoreTakeRecursive(g_hVarMutex, portMAX_DELAY)
#define UNLOCK_VAR()  xSemaphoreGiveRecursive(g_hVarMutex)

/* Coils: D01 sensor1 normal, D02 sensor1 blowback, D03 sensor2 normal, D04 sensor2 blowback */
#define REG_D01      1
#define REG_D02      2
#define REG_D03      3
#define REG_D04      4
#define REG_D05      5
#define REG_DXX      REG_D04

#define SLAVE_REG_START  40001

/* ==================== Common registers (shared / output block) ==================== */
#define SLAVE_REG_NOX_OUTPUT      40001   /* NOx output (float, 2 regs) */
#define SLAVE_REG_O2_OUTPUT       40003   /* O2 output (float, 2 regs) */
#define SLAVE_REG_OUTPUT_CH_STATUS 40005  /* Output channel status (R, u16) */
#define SLAVE_REG_ALARM_NOX_HI    40006   /* Alarm NOx high threshold (float, 2 regs) */
#define SLAVE_REG_ALARM_O2_LO     40008   /* Alarm O2 low threshold (float, 2 regs) */
#define SLAVE_REG_MA_NOX          40010   /* 4-20mA NOx raw (u16) */
#define SLAVE_REG_MA_O2           40011   /* 4-20mA O2 raw (u16) */
#define SLAVE_REG_WORK_MODE       40012   /* Work mode only: 0=single 1=primary-backup 2=fusion. Write: low byte mode; single uses high byte 0/256=ch0/ch1. */
#define SLAVE_REG_OUTPUT_SENSOR   40013   /* P35 R-only: 0b01=S0, 0b10=S1, 0b11=fusion, 0b00=fault */
#define COMMON_REG_END            40013   /* End address of common block */

/* ==================== Per-sensor registers (same layout each, 38 regs per channel) ==================== */
/* Layout order: power_on | live NOx, live O2, status | seg1/seg2/P2/P3 | cal/blow ... */
#define SENSOR_BASE_1    40014
#define SENSOR_BASE_2    40053
#define SENSOR_REG_COUNT 39

/* Sensor channel 1 addresses (40014-40051), first reg = power on (R/W, GPIO control reserved) */
#define SLAVE_REG_S1_POWER        40014   /* R/W: 0=off 1=on, drives reserved GPIO when defined */
#define SLAVE_REG_S1_LIVE_NOX    40015
#define SLAVE_REG_S1_LIVE_O2     40017
#define SLAVE_REG_S1_STATUS      40019
#define SLAVE_REG_S1_SEG1_NOX_A  40020
#define SLAVE_REG_S1_SEG1_NOX_B  40022
#define SLAVE_REG_S1_SEG1_O2_A   40024
#define SLAVE_REG_S1_SEG1_O2_B   40026
#define SLAVE_REG_S1_SEG2_NOX_A  40028
#define SLAVE_REG_S1_SEG2_NOX_B  40030
#define SLAVE_REG_S1_SEG2_O2_A   40032
#define SLAVE_REG_S1_SEG2_O2_B   40034
#define SLAVE_REG_S1_P2_NOX      40036
#define SLAVE_REG_S1_P2_O2       40038
#define SLAVE_REG_S1_P3_NOX      40040
#define SLAVE_REG_S1_P3_O2       40042
#define SLAVE_REG_S1_NOX_CAL_TRIG 40044
#define SLAVE_REG_S1_NOX_PT_SEL  40045
#define SLAVE_REG_S1_O2_CAL_TRIG  40046
#define SLAVE_REG_S1_O2_PT_SEL    40047
#define SLAVE_REG_S1_BLOW_INT    40048
#define SLAVE_REG_S1_BLOW_DUR    40049
#define SLAVE_REG_S1_BLOW_STATUS 40050
#define SLAVE_REG_S1_BLOW_CD     40051   /* R: idle=sec to next blow; blowing=sec left */
#define SLAVE_REG_S1_BLOW_CMD    40052   /* 40014-40052 = 39 regs S1 */

/* Sensor channel 2 addresses (40053-40091), same layout as channel 1 */
#define SLAVE_REG_S2_POWER        40053
#define SLAVE_REG_S2_LIVE_NOX    40054
#define SLAVE_REG_S2_LIVE_O2     40056
#define SLAVE_REG_S2_STATUS      40058
#define SLAVE_REG_S2_SEG1_NOX_A  40059
#define SLAVE_REG_S2_SEG1_NOX_B  40061
#define SLAVE_REG_S2_SEG1_O2_A   40063
#define SLAVE_REG_S2_SEG1_O2_B   40065
#define SLAVE_REG_S2_SEG2_NOX_A  40067
#define SLAVE_REG_S2_SEG2_NOX_B  40069
#define SLAVE_REG_S2_SEG2_O2_A   40071
#define SLAVE_REG_S2_SEG2_O2_B   40073
#define SLAVE_REG_S2_P2_NOX      40075
#define SLAVE_REG_S2_P2_O2       40077
#define SLAVE_REG_S2_P3_NOX      40079
#define SLAVE_REG_S2_P3_O2       40081
#define SLAVE_REG_S2_NOX_CAL_TRIG 40083
#define SLAVE_REG_S2_NOX_PT_SEL  40084
#define SLAVE_REG_S2_O2_CAL_TRIG  40085
#define SLAVE_REG_S2_O2_PT_SEL    40086
#define SLAVE_REG_S2_BLOW_INT    40087
#define SLAVE_REG_S2_BLOW_DUR    40088
#define SLAVE_REG_S2_BLOW_STATUS 40089
#define SLAVE_REG_S2_BLOW_CD     40090   /* R: idle=sec to next blow; blowing=sec left */
#define SLAVE_REG_S2_BLOW_CMD    40091   /* 40053-40091 = 39 regs S2 */

#define RSP_OK              0
#define RSP_ERR_CMD         0x01
#define RSP_ERR_REG_ADDR    0x02
#define RSP_ERR_VALUE       0x03
#define RSP_ERR_WRITE       0x04

#define S_RX_BUF_SIZE       60
/* 03H read max 100 regs => 3 + 200 + 2 CRC; use 256 for margin */
#define S_TX_BUF_SIZE       256

typedef struct {
    uint8_t RxBuf[S_RX_BUF_SIZE];
    uint8_t RxCount;
    uint8_t RspCode;
    uint8_t TxBuf[S_TX_BUF_SIZE];
    uint8_t TxCount;
} MODS_T;

/* Per-channel sensor register mirror (same struct for S1 and S2) */
typedef struct {
    uint16_t power_on;   /* 0=off 1=on, drives reserved sensor power GPIO when defined */
    float    live_nox, live_o2;
    uint16_t status;
    float    seg1_nox_a, seg1_nox_b, seg1_o2_a, seg1_o2_b;
    float    seg2_nox_a, seg2_nox_b, seg2_o2_a, seg2_o2_b;
    float    p2_nox, p2_o2, p3_nox, p3_o2;
    uint16_t nox_cal_trig, nox_pt_sel, o2_cal_trig, o2_pt_sel;
    uint16_t blow_interval, blow_duration, blow_status, blow_countdown, blow_cmd;
} SensorRegs_t;

typedef struct {
    /* Common: NOx/O2 outputs, channel status, alarm thresholds, 4-20mA raw, work mode, coils */
    float    nox_output, o2_output;
    uint16_t output_ch_status;
    float    alarm_nox_hi, alarm_o2_lo;
    uint16_t ma_nox, ma_o2, work_mode;
    uint16_t coil_d01, coil_d02, coil_d03, coil_d04;

    SensorRegs_t S1, S2;
} VAR_T;

void MODS_Poll(void);
void MODS_ReciveNew(uint8_t _byte);
extern void (*s_TIM_CallBack1)(void);
extern MODS_T g_tModS;
extern VAR_T g_tVar;
extern SemaphoreHandle_t MODRx_SemaphoreHandle;
extern void Start_Receive(void);

/* Coils */
extern void Var_Write_D01(uint16_t value);
extern uint16_t Var_Read_D01(void);
extern void Var_Write_D02(uint16_t value);
extern uint16_t Var_Read_D02(void);
extern void Var_Write_D03(uint16_t value);
extern uint16_t Var_Read_D03(void);
extern void Var_Write_D04(uint16_t value);
extern uint16_t Var_Read_D04(void);

/* Common register accessors */
extern float Var_Read_NoxOutput(void);
extern float Var_Read_O2Output(void);
extern uint16_t Var_Read_OutputChStatus(void);
extern void Var_Write_NoxOutput(float value);
extern void Var_Write_O2Output(float value);
extern void Var_Write_OutputChStatus(uint16_t value);

extern void Var_Write_AlarmNoxHi(float value);
extern float Var_Read_AlarmNoxHi(void);
extern void Var_Write_AlarmO2Lo(float value);
extern float Var_Read_AlarmO2Lo(void);
extern void Var_Write_MaNox(uint16_t value);
extern uint16_t Var_Read_MaNox(void);
extern void Var_Write_MaO2(uint16_t value);
extern uint16_t Var_Read_MaO2(void);
extern void Var_Write_WorkMode(uint16_t value);
extern uint16_t Var_Read_WorkMode(void);
/** P34: mode0 写 0/256=ch0/ch1; mode1 写 1/257=ch0/ch1 主. Read returns channel index 0/1. */
extern uint8_t Var_Read_SingleChannelIndex(void);
/** P35 output sensor (40013, R-only): 0b01=S0 0b10=S1 0b11=fusion 0b00=fault. */
extern uint8_t Var_Read_OutputSensorReg(void);

/* Per-channel sensor accessors; ch=0 or 1, same layout as common block */
extern uint16_t Var_Read_SensorPowerOn(uint8_t ch);
extern void Var_Write_SensorPowerOn(uint8_t ch, uint16_t v);

extern float  Var_Read_SensorLiveNox(uint8_t ch);
extern float  Var_Read_SensorLiveO2(uint8_t ch);
extern uint16_t Var_Read_SensorStatus(uint8_t ch);
extern void Var_Write_SensorLiveNox(uint8_t ch, float v);
extern void Var_Write_SensorLiveO2(uint8_t ch, float v);
extern void Var_Write_SensorStatus(uint8_t ch, uint16_t v);

extern float  Var_Read_SensorSeg1NoxA(uint8_t ch);
extern float  Var_Read_SensorSeg1NoxB(uint8_t ch);
extern float  Var_Read_SensorSeg1O2A(uint8_t ch);
extern float  Var_Read_SensorSeg1O2B(uint8_t ch);
extern void Var_Write_SensorSeg1NoxA(uint8_t ch, float v);
extern void Var_Write_SensorSeg1NoxB(uint8_t ch, float v);
extern void Var_Write_SensorSeg1O2A(uint8_t ch, float v);
extern void Var_Write_SensorSeg1O2B(uint8_t ch, float v);

extern float  Var_Read_SensorSeg2NoxA(uint8_t ch);
extern float  Var_Read_SensorSeg2NoxB(uint8_t ch);
extern float  Var_Read_SensorSeg2O2A(uint8_t ch);
extern float  Var_Read_SensorSeg2O2B(uint8_t ch);
extern void Var_Write_SensorSeg2NoxA(uint8_t ch, float v);
extern void Var_Write_SensorSeg2NoxB(uint8_t ch, float v);
extern void Var_Write_SensorSeg2O2A(uint8_t ch, float v);
extern void Var_Write_SensorSeg2O2B(uint8_t ch, float v);

extern float  Var_Read_SensorP2Nox(uint8_t ch);
extern float  Var_Read_SensorP2O2(uint8_t ch);
extern float  Var_Read_SensorP3Nox(uint8_t ch);
extern float  Var_Read_SensorP3O2(uint8_t ch);
extern void Var_Write_SensorP2Nox(uint8_t ch, float v);
extern void Var_Write_SensorP2O2(uint8_t ch, float v);
extern void Var_Write_SensorP3Nox(uint8_t ch, float v);
extern void Var_Write_SensorP3O2(uint8_t ch, float v);

extern uint16_t Var_Read_SensorNoxCalTrig(uint8_t ch);
extern uint16_t Var_Read_SensorNoxPtSel(uint8_t ch);
extern uint16_t Var_Read_SensorO2CalTrig(uint8_t ch);
extern uint16_t Var_Read_SensorO2PtSel(uint8_t ch);
extern void Var_Write_SensorNoxCalTrig(uint8_t ch, uint16_t v);
extern void Var_Write_SensorNoxPtSel(uint8_t ch, uint16_t v);
extern void Var_Write_SensorO2CalTrig(uint8_t ch, uint16_t v);
extern void Var_Write_SensorO2PtSel(uint8_t ch, uint16_t v);

extern uint16_t Var_Read_SensorBlowInterval(uint8_t ch);
extern uint16_t Var_Read_SensorBlowDuration(uint8_t ch);
extern uint16_t Var_Read_SensorBlowStatus(uint8_t ch);
extern uint16_t Var_Read_SensorBlowCountdown(uint8_t ch);
extern uint16_t Var_Read_SensorBlowCmd(uint8_t ch);
extern void Var_Write_SensorBlowInterval(uint8_t ch, uint16_t v);
extern void Var_Write_SensorBlowDuration(uint8_t ch, uint16_t v);
extern void Var_Write_SensorBlowCmd(uint8_t ch, uint16_t v);

/* Blowback/alarm config readback */
extern void Var_Read_BlowbackCfg(uint16_t *p24, uint16_t *p25);
extern void Var_Read_AlarmCfg(float *p12, float *p13);
extern void Var_Update_SensorCore(float nox, float o2, uint16_t state);
extern void Var_Update_CalibPoint(float p18, float p19, float p20, float p21);

#endif
