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

/* Coils D01-D04 are legacy variables only; valve GPIO is controlled by holding registers. */
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
#define SLAVE_REG_WORK_MODE       40012   /* Read mode only. Write: low byte mode; high byte selects target/primary ch0/ch1/ch2 for mode 0/1. */
#define SLAVE_REG_OUTPUT_SENSOR   40013   /* P35 R-only: 0b01=S1/ch0, 0b10=S2/ch1, 0b11=fusion, 0b100=S3/ch2, 0b00=fault */
#define COMMON_REG_END            40013   /* End address of common block */

/* ==================== Per-sensor registers (same layout each, 39 + 3 valve regs = 42 per channel) ==================== */
/* Layout: power_on | live NOx/O2/status | seg1/seg2/P2/P3 | cal/blow | valve_normal, valve_blow, valve_cal */
#define SENSOR_BASE_1    40014
#define SENSOR_BASE_2    40056
#define SENSOR_REG_COUNT 42

/* Sensor channel 1 (40014-40055): 39 + 3 valve (J1=正常抽气,J2=反吹,J3=校准) */
#define SLAVE_REG_S1_POWER        40014
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
#define SLAVE_REG_S1_NOX_PT_SEL  40044   /* NOx 点选择（0/1/2） */
#define SLAVE_REG_S1_NOX_CAL_TRIG 40045  /* NOx 标定触发 */
#define SLAVE_REG_S1_O2_CAL_TRIG  40046
#define SLAVE_REG_S1_O2_PT_SEL    40047
#define SLAVE_REG_S1_BLOW_INT    40048
#define SLAVE_REG_S1_BLOW_DUR    40049
#define SLAVE_REG_S1_BLOW_STATUS 40050
#define SLAVE_REG_S1_BLOW_CD     40051
#define SLAVE_REG_S1_BLOW_CMD    40052
#define SLAVE_REG_S1_VALVE_NORMAL 40053   /* R/W: 0=关 1=开, J1_IN(PA4) 正常抽气检测 */
#define SLAVE_REG_S1_VALVE_BLOW   40054   /* R/W: 0=关 1=开, J2_IN(PA3) 反吹；打开时关闭同路正常抽气 */
#define SLAVE_REG_S1_VALVE_CAL    40055   /* R/W: 0=关 1=开, J3_IN(PA1) 校准 */

/* Sensor channel 2 (40056-40097), same layout + 3 valve (J4,J5,J6) */
#define SLAVE_REG_S2_POWER        40056
#define SLAVE_REG_S2_LIVE_NOX    40057
#define SLAVE_REG_S2_LIVE_O2     40059
#define SLAVE_REG_S2_STATUS      40061
#define SLAVE_REG_S2_SEG1_NOX_A  40062
#define SLAVE_REG_S2_SEG1_NOX_B  40064
#define SLAVE_REG_S2_SEG1_O2_A   40066
#define SLAVE_REG_S2_SEG1_O2_B   40068
#define SLAVE_REG_S2_SEG2_NOX_A  40070
#define SLAVE_REG_S2_SEG2_NOX_B  40072
#define SLAVE_REG_S2_SEG2_O2_A   40074
#define SLAVE_REG_S2_SEG2_O2_B   40076
#define SLAVE_REG_S2_P2_NOX      40078
#define SLAVE_REG_S2_P2_O2       40080
#define SLAVE_REG_S2_P3_NOX      40082
#define SLAVE_REG_S2_P3_O2       40084
#define SLAVE_REG_S2_NOX_PT_SEL  40086   /* NOx 点选择（0/1/2） */
#define SLAVE_REG_S2_NOX_CAL_TRIG 40087  /* NOx 标定触发 */
#define SLAVE_REG_S2_O2_CAL_TRIG  40088
#define SLAVE_REG_S2_O2_PT_SEL    40089
#define SLAVE_REG_S2_BLOW_INT    40090
#define SLAVE_REG_S2_BLOW_DUR    40091
#define SLAVE_REG_S2_BLOW_STATUS 40092
#define SLAVE_REG_S2_BLOW_CD     40093
#define SLAVE_REG_S2_BLOW_CMD    40094
#define SLAVE_REG_S2_VALVE_NORMAL 40095   /* J4_IN(PA0) */
#define SLAVE_REG_S2_VALVE_BLOW   40096   /* J5_IN(PC3) */
#define SLAVE_REG_S2_VALVE_CAL    40097   /* J6_IN(PB3) */

/* Sensor channel 3 (40098-40139), same layout + 3 valve (J7,J8,J9) */
#define SENSOR_BASE_3      40098
#define SLAVE_REG_S3_POWER        40098
#define SLAVE_REG_S3_LIVE_NOX    40099
#define SLAVE_REG_S3_LIVE_O2     40101
#define SLAVE_REG_S3_STATUS      40103
#define SLAVE_REG_S3_SEG1_NOX_A  40104
#define SLAVE_REG_S3_SEG1_NOX_B  40106
#define SLAVE_REG_S3_SEG1_O2_A   40108
#define SLAVE_REG_S3_SEG1_O2_B   40110
#define SLAVE_REG_S3_SEG2_NOX_A  40112
#define SLAVE_REG_S3_SEG2_NOX_B  40114
#define SLAVE_REG_S3_SEG2_O2_A   40116
#define SLAVE_REG_S3_SEG2_O2_B   40118
#define SLAVE_REG_S3_P2_NOX      40120
#define SLAVE_REG_S3_P2_O2       40122
#define SLAVE_REG_S3_P3_NOX      40124
#define SLAVE_REG_S3_P3_O2       40126
#define SLAVE_REG_S3_NOX_PT_SEL  40128   /* NOx 点选择（0/1/2） */
#define SLAVE_REG_S3_NOX_CAL_TRIG 40129  /* NOx 标定触发 */
#define SLAVE_REG_S3_O2_CAL_TRIG  40130
#define SLAVE_REG_S3_O2_PT_SEL    40131
#define SLAVE_REG_S3_BLOW_INT    40132
#define SLAVE_REG_S3_BLOW_DUR    40133
#define SLAVE_REG_S3_BLOW_STATUS 40134
#define SLAVE_REG_S3_BLOW_CD     40135
#define SLAVE_REG_S3_BLOW_CMD    40136
#define SLAVE_REG_S3_VALVE_NORMAL 40137   /* J7_IN(PB4) */
#define SLAVE_REG_S3_VALVE_BLOW   40138   /* J8_IN(PB5) */
#define SLAVE_REG_S3_VALVE_CAL    40139   /* J9_IN(PB8) */

/* 6-channel 4-20mA module readback/source mirror: S1 NOx/O2, S2 NOx/O2, S3 NOx/O2 */
#define SLAVE_REG_MA_S1_NOX       40140
#define SLAVE_REG_MA_S1_O2        40141
#define SLAVE_REG_MA_S2_NOX       40142
#define SLAVE_REG_MA_S2_O2        40143
#define SLAVE_REG_MA_S3_NOX       40144
#define SLAVE_REG_MA_S3_O2        40145

/* 保持寄存器映射 40001..40145 共 145 字；Modbus 03H 单次最多读 MODBUS_FC03_MAX_REGS（125）——主站须分两次轮询，例如 40001×125 + 40126×20。 */
#define MODBUS_FC03_MAX_REGS           125u
#define SLAVE_HOLDING_REG_LAST         SLAVE_REG_MA_S3_O2
#define SLAVE_HOLDING_REG_TOTAL        145u

#define RSP_OK              0
#define RSP_ERR_CMD         0x01
#define RSP_ERR_REG_ADDR    0x02
#define RSP_ERR_VALUE       0x03
#define RSP_ERR_WRITE       0x04

#define S_RX_BUF_SIZE       60
/* 03H 最多 125 字 ×2 字节 + 地址/功能/字节数/CRC ≈ 255，留余量 */
#define S_TX_BUF_SIZE       260

typedef struct {
    /** UART 字节流缓冲，仅 MODS_ReciveNew 写入；超时后新帧也从 [0] 起累加 */
    uint8_t RxBuf[S_RX_BUF_SIZE];
    /** 仅 ISR 收包与 pending 恢复；与已完成帧长度无关 */
    uint8_t RxCount;
    /** 3.5 字符超时瞬间从 RxBuf 拷贝的完整一帧；CRC/功能码解析只读此处，不再拷回 RxBuf */
    uint8_t RxFrameSnap[S_RX_BUF_SIZE];
    /** RxFrameSnap 中有效字节数（快照长度） */
    uint8_t RxFrameLen;
    uint8_t RspCode;
    uint8_t TxBuf[S_TX_BUF_SIZE];
    uint8_t TxCount;
} MODS_T;

/* Per-channel sensor register mirror (same struct for S1/S2/S3) */
typedef struct {
    uint16_t power_on;
    float    live_nox, live_o2;
    uint16_t status;
    float    seg1_nox_a, seg1_nox_b, seg1_o2_a, seg1_o2_b;
    float    seg2_nox_a, seg2_nox_b, seg2_o2_a, seg2_o2_b;
    float    p2_nox, p2_o2, p3_nox, p3_o2;
    uint16_t nox_cal_trig, nox_pt_sel, o2_cal_trig, o2_pt_sel;
    uint16_t blow_interval, blow_duration, blow_status, blow_countdown, blow_cmd;
    uint16_t valve_normal, valve_blow, valve_cal;  /* 0=关 1=开, 对应 J1-J9 */
} SensorRegs_t;

typedef struct {
    /* Common: NOx/O2 outputs, channel status, alarm thresholds, 4-20mA raw, work mode, coils */
    float    nox_output, o2_output;
    uint16_t output_ch_status;
    float    alarm_nox_hi, alarm_o2_lo;
    uint16_t ma_nox, ma_o2, work_mode;
    uint16_t ma_s1_nox, ma_s1_o2, ma_s2_nox, ma_s2_o2, ma_s3_nox, ma_s3_o2;
    uint16_t coil_d01, coil_d02, coil_d03, coil_d04;

    SensorRegs_t S1, S2, S3;
} VAR_T;

void MODS_Poll(void);
void MODS_ReciveNew(uint8_t _byte);
extern void (*s_TIM_CallBack1)(void);
extern MODS_T g_tModS;
extern VAR_T g_tVar;
extern SemaphoreHandle_t MODRx_SemaphoreHandle;
extern volatile uint32_t g_mods_rx_timeout_count;
extern volatile uint32_t g_mods_frame_ok_count;
extern volatile uint32_t g_mods_short_frame_count;
extern volatile uint32_t g_mods_crc_error_count;
extern volatile uint32_t g_mods_addr_miss_count;
extern volatile uint32_t g_mods_tx_start_count;
extern volatile uint32_t g_mods_tx_busy_count;
extern volatile uint32_t g_mods_tx_fail_count;
extern volatile uint8_t g_mods_last_frame_len;
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
extern void Var_Write_MaSensorNox(uint8_t ch, uint16_t value);
extern uint16_t Var_Read_MaSensorNox(uint8_t ch);
extern void Var_Write_MaSensorO2(uint8_t ch, uint16_t value);
extern uint16_t Var_Read_MaSensorO2(uint8_t ch);
extern void Var_Write_WorkMode(uint16_t value);
extern uint16_t Var_Read_WorkMode(void);
/** P34: mode0 写 0/256=ch0/ch1; mode1 写 1/257=ch0/ch1 主. Read returns channel index 0/1. */
extern uint8_t Var_Read_SingleChannelIndex(void);
/** P35 output sensor (40013, R-only): 0b01=S1/ch0 0b10=S2/ch1 0b11=fusion 0b100=S3/ch2 0b00=fault. */
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

/* 阀门保持寄存器：v=0 正常抽气 1 反吹 2 校准；写时驱动 J1-J9，并执行同路互锁 */
extern uint16_t Var_Read_SensorValve(uint8_t ch, uint8_t v);
extern void Var_Write_SensorValve(uint8_t ch, uint8_t v, uint16_t value);
/** 反吹结束后由 blowback 调用，将 valve_blow 寄存器写回对应 GPIO */
extern void Modbus_ApplySensorValveBlowToGPIO(uint8_t ch);
/** 反吹结束后将 valve_normal（抽气）寄存器写回 GPIO */
extern void Modbus_ApplySensorValveNormalToGPIO(uint8_t ch);
/** 反吹开始时仅拉低抽气 GPIO，不改变 g_tVar.valve_normal */
extern void Modbus_ForceSensorNormalValveOff(uint8_t ch);
/** 清除手动抽气覆盖；反吹/校准等安全联锁介入时调用 */
extern void Modbus_ClearManualSuctionOverride(uint8_t ch);
/**
 * 未手动覆盖时，状态字 0x1FF、且非反吹、且校准阀关 -> 抽气继电器置 1；否则置 0。
 * 同路非反吹、非校准时，手动写正常抽气阀为 1 可绕过 0x1FF 条件；反吹/校准仍会强制关闭并清除覆盖。
 */
extern void Modbus_AutoSuctionValvesUpdate(void);

/* Blowback/alarm config readback */
extern void Var_Read_BlowbackCfg(uint16_t *p24, uint16_t *p25);
extern void Var_Read_AlarmCfg(float *p12, float *p13);
extern void Var_Update_SensorCore(float nox, float o2, uint16_t state);
extern void Var_Update_CalibPoint(float p18, float p19, float p20, float p21);

#endif
