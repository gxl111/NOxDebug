/*
 * modbus_slave.h - Modbus RTU slave (holding registers + coils).
 * ͨ�üĴ���: �� NOx/O2 �����4-20mA��ģʽ��������ֵ��
 * �������Ĵ���: ��·��������ȫ�Գƣ������빦��һ�¡�
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

/* ==================== ͨ�üĴ����������¼��ࣩ ==================== */
#define SLAVE_REG_NOX_OUTPUT      40001   /* NOx ��� (float, 2 regs) */
#define SLAVE_REG_O2_OUTPUT       40003   /* O2 ��� (float, 2 regs) */
#define SLAVE_REG_OUTPUT_CH_STATUS 40005  /* ��ǰ���ͨ��״̬ (R, u16) */
#define SLAVE_REG_ALARM_NOX_HI    40006   /* ���� NOx ���� (float, 2 regs) */
#define SLAVE_REG_ALARM_O2_LO     40008   /* ���� O2 ���� (float, 2 regs) */
#define SLAVE_REG_MA_NOX          40010   /* 4-20mA NOx �� (u16) */
#define SLAVE_REG_MA_O2           40011   /* 4-20mA O2 �� (u16) */
#define SLAVE_REG_WORK_MODE       40012   /* ����ģʽ (u16): ���ֽ� 0=��· 1=���� 2=�ںϣ���·ʱ���ֽ�=ͨ�� 0/1���� 0x0100 ��ʾͨ��1 */
#define COMMON_REG_END            40012   /* ͨ�ý�����ַ */

/* ==================== �������飨��·��ȫһ�£�ÿ�� 38 ���Ĵ�����ַ�� ==================== */
/* ����˳��: ʵʱNOx, ʵʱO2, ״̬ | �궨��1 NOx/O2 | �궨��2 NOx/O2 | �궨��2/3 NOx/O2 | �궨���� NOx/O2 | ���� ... */
#define SENSOR_BASE_1    40013
#define SENSOR_BASE_2    40051
#define SENSOR_REG_COUNT 38

/* ������1 ��ַ (40013-40050) */
#define SLAVE_REG_S1_LIVE_NOX    40013
#define SLAVE_REG_S1_LIVE_O2     40015
#define SLAVE_REG_S1_STATUS      40017
#define SLAVE_REG_S1_SEG1_NOX_A  40018
#define SLAVE_REG_S1_SEG1_NOX_B  40020
#define SLAVE_REG_S1_SEG1_O2_A   40022
#define SLAVE_REG_S1_SEG1_O2_B   40024
#define SLAVE_REG_S1_SEG2_NOX_A  40026
#define SLAVE_REG_S1_SEG2_NOX_B  40028
#define SLAVE_REG_S1_SEG2_O2_A   40030
#define SLAVE_REG_S1_SEG2_O2_B   40032
#define SLAVE_REG_S1_P2_NOX      40034
#define SLAVE_REG_S1_P2_O2       40036
#define SLAVE_REG_S1_P3_NOX      40038
#define SLAVE_REG_S1_P3_O2       40040
#define SLAVE_REG_S1_NOX_CAL_TRIG 40042
#define SLAVE_REG_S1_NOX_PT_SEL  40043
#define SLAVE_REG_S1_O2_CAL_TRIG  40044
#define SLAVE_REG_S1_O2_PT_SEL    40045
#define SLAVE_REG_S1_BLOW_INT    40046
#define SLAVE_REG_S1_BLOW_DUR    40047
#define SLAVE_REG_S1_BLOW_STATUS 40048
#define SLAVE_REG_S1_BLOW_CD     40049
#define SLAVE_REG_S1_BLOW_CMD    40050

/* ������2 ��ַ (40051-40088)���봫����1 һһ��Ӧ */
#define SLAVE_REG_S2_LIVE_NOX    40051
#define SLAVE_REG_S2_LIVE_O2     40053
#define SLAVE_REG_S2_STATUS      40055
#define SLAVE_REG_S2_SEG1_NOX_A  40056
#define SLAVE_REG_S2_SEG1_NOX_B  40058
#define SLAVE_REG_S2_SEG1_O2_A   40060
#define SLAVE_REG_S2_SEG1_O2_B   40062
#define SLAVE_REG_S2_SEG2_NOX_A  40064
#define SLAVE_REG_S2_SEG2_NOX_B  40066
#define SLAVE_REG_S2_SEG2_O2_A   40068
#define SLAVE_REG_S2_SEG2_O2_B   40070
#define SLAVE_REG_S2_P2_NOX      40072
#define SLAVE_REG_S2_P2_O2       40074
#define SLAVE_REG_S2_P3_NOX      40076
#define SLAVE_REG_S2_P3_O2       40078
#define SLAVE_REG_S2_NOX_CAL_TRIG 40080
#define SLAVE_REG_S2_NOX_PT_SEL  40081
#define SLAVE_REG_S2_O2_CAL_TRIG  40082
#define SLAVE_REG_S2_O2_PT_SEL    40083
#define SLAVE_REG_S2_BLOW_INT    40084
#define SLAVE_REG_S2_BLOW_DUR    40085
#define SLAVE_REG_S2_BLOW_STATUS 40086
#define SLAVE_REG_S2_BLOW_CD     40087
#define SLAVE_REG_S2_BLOW_CMD    40088

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

/* ��·�������Ĵ�������·�ṹһ�£� */
typedef struct {
    float    live_nox, live_o2;
    uint16_t status;
    float    seg1_nox_a, seg1_nox_b, seg1_o2_a, seg1_o2_b;
    float    seg2_nox_a, seg2_nox_b, seg2_o2_a, seg2_o2_b;
    float    p2_nox, p2_o2, p3_nox, p3_o2;
    uint16_t nox_cal_trig, nox_pt_sel, o2_cal_trig, o2_pt_sel;
    uint16_t blow_interval, blow_duration, blow_status, blow_countdown, blow_cmd;
} SensorRegs_t;

typedef struct {
    /* ͨ��: NOx/O2 ��������ͨ��״̬��������ֵ��4-20mA������ģʽ����Ȧ */
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

/* ͨ�üĴ��� */
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
/** P34/work_mode: low byte = work mode (0/1/2), high byte = single-channel index (0 or 1) when mode=0.
 *  Single ch0: write 0, single ch1: write 0x0100 (256). */
extern uint8_t Var_Read_SingleChannelIndex(void);

/* �������Ĵ�������ͨ�� ch=0 �� 1 ���ʣ���·�Ĵ��������빦��һ�� */
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

/* ���ݾɴ���������� */
extern void Var_Read_BlowbackCfg(uint16_t *p24, uint16_t *p25);
extern void Var_Read_AlarmCfg(float *p12, float *p13);
extern void Var_Update_SensorCore(float nox, float o2, uint16_t state);
extern void Var_Update_CalibPoint(float p18, float p19, float p20, float p21);

#endif
