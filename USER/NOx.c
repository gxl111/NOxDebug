/**
 * @file    NOx.c
 * @brief   NOx tasks: J1939 receive/handle (per channel), strategy (single/primary_backup/fusion),
 *          default task (calibration, alarm, blowback, OLED), Modbus slave init.
 */
#include "NOx.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "modbus_slave.h"
#include "modbus_flash.h"
#include "modbus_host.h"
#include "blowback.h"
#include "calibration.h"
#include "alarm.h"
#include "oled.h"
#include "sdcard.h"
#include "app_config.h"
#include "nox_channel.h"
#include "J1939.H"
#include "mcp2515_spi_can.h"

extern uint32_t time_1s;

void Register_Init(void);
void AfterFlash_Init(void);

/* J1939 Tx = heater command (one frame for both sensors via Byte7 Start-Code). */
static J1939_MESSAGE TxMessage;
/* Heater payload: Byte7 = ATI1|ATO1|ATI2|ATO2 dew point (0x55 = both banks). */
static const uint8_t HeaterData[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, J1939_HEATER_PAYLOAD_TAIL };

SemaphoreHandle_t g_hVarMutex = NULL;

void TxMsg_Init(J1939_MESSAGE *TxMsgPtr)
{
    TxMsgPtr->Mxe.Priority    = 0x06;
    TxMsgPtr->Mxe.PDUFormat   = 0xFE;
    TxMsgPtr->Mxe.SourceAddress = J1939_Address;
    TxMsgPtr->Mxe.DataLength = 8;
    TxMsgPtr->Mxe.PDUSpecific = 0xDF;
    TxMsgPtr->Mxe.DataPage   = 0;
    TxMsgPtr->Mxe.Res        = 0;
    TxMsgPtr->Mxe.RTR       = 0;
    for (int i = 0; i < 8; i++)
        TxMsgPtr->Mxe.Data[i] = HeaterData[i];
}

void hexArrayToString(const j1939_uint8_t *array, size_t length, char *result)
{
    result[0] = '\0';
    for (size_t i = 0; i < length; i++) {
        char tmp[3];
        snprintf(tmp, sizeof(tmp), "%02X", array[i]);
        strcat(result, tmp);
    }
}

/* Update one channel from J1939 payload; then strategy updates P01/P02/P07 and per-channel (NOx, O2, status). */
static void NOx_HandleOne(J1939_MESSAGE *RxMsgPtr, uint8_t ch_index)
{
    NoxChannel_UpdateFromCan(ch_index, RxMsgPtr->Mxe.Data);
}

/* Update Modbus per-channel readings: S1/S2 写入 Var；S3（ch2）无 Modbus 块，仅参与策略. */
static void NOx_UpdatePerChannelRegs(void)
{
    LOCK_VAR();
    for (uint8_t ch = 0; ch < 2u; ch++) {
        NoxChannel_t *c = &g_noxChannels[ch];
        Var_Write_SensorLiveNox(ch, c->nox_ppm);
        Var_Write_SensorLiveO2(ch, c->o2_pct);
        Var_Write_SensorStatus(ch, c->state);
    }
    UNLOCK_VAR();
}

/* ----- NOx receive task: dequeue J1939_RX_ITEM (channel+msg), update channel, strategy, 4-20mA, heater ----- */
void NOxReceive(void *argument)
{
    J1939_RX_ITEM item;
    TxMsg_Init(&TxMessage);

    for (;;) {
        if (xQueueReceive(Rx_QueueHandle, &item, pdMS_TO_TICKS(100)) == pdPASS) {
            NOx_HandleOne(&item.msg, item.channel_index);
        }
        /* 第二路 CAN（MCP2515）接收：SA 0x52 → 通道 2（Electrical Interface 18F00F52h） */
        {
            MCP2515_CAN_Frame_t rx;
            while (MCP2515_Receive(&rx) == 1) {
                if (rx.is_ext_id && rx.id == 0x18F00F52u && rx.len >= 8) {
                    J1939_RX_ITEM can2_item;
                    can2_item.channel_index = 2u;
                    can2_item.msg.Array[0] = (j1939_uint8_t)(rx.id >> 24);
                    can2_item.msg.Array[1] = (j1939_uint8_t)(rx.id >> 16);
                    can2_item.msg.Array[2] = (j1939_uint8_t)(rx.id >> 8);
                    can2_item.msg.Array[3] = (j1939_uint8_t)(rx.id);
                    can2_item.msg.Mxe.DataLength = 8;
                    for (int i = 0; i < 8; i++)
                        can2_item.msg.Mxe.Data[i] = rx.data[i];
                    xQueueSend(Rx_QueueHandle, &can2_item, 0);
                }
            }
        }

        /* P34: mode 0 single (high byte = ch), mode 1 主从 (high byte = 主 ch), mode 2 fusion (no 主). */
        {
            uint16_t mode_u = Var_Read_WorkMode();
            if (mode_u <= (uint16_t)NOX_MODE_FUSION)
                NoxChannel_SetWorkMode((NoxWorkMode_t)mode_u);
            NoxChannel_SetSingleChannelIndex(Var_Read_SingleChannelIndex());
        }

        /* Current output for P01/P02(P07) and single 4-20mA: NOx and O2 from strategy. */
        {
            float nox_out, o2_out;
            uint16_t state_out;
            NoxChannel_GetCurrentOutput(&nox_out, &o2_out, &state_out);
            NOx_ppm = nox_out;
            O2_pct  = o2_out;
            Var_Update_SensorCore(nox_out, o2_out, state_out);
            NOx_UpdatePerChannelRegs();
            NoxSensor_To4_20mA(nox_out, o2_out, electricity_data_buf);
        }

        /* OLED: S1/S2/S3（S3=第二路CAN SA 0x52）+ 当前输出 */
        {
            uint8_t buf[28];
            float n1 = Var_Read_SensorLiveNox(0), o1 = Var_Read_SensorLiveO2(0);
            float n2 = Var_Read_SensorLiveNox(1), o2 = Var_Read_SensorLiveO2(1);
            uint16_t st1 = Var_Read_SensorStatus(0), st2 = Var_Read_SensorStatus(1);
            NoxChannel_t *c3 = &g_noxChannels[2];
            snprintf((char *)buf, sizeof(buf), "S1 NOx %5.2f O2 %5.2f     ", (double)n1, (double)o1);
            OLED_PrintASCIIString(0, 10, (char *)buf, &afont8x6, OLED_COLOR_NORMAL);
            snprintf((char *)buf, sizeof(buf), "S2 NOx %5.2f O2 %5.2f     ", (double)n2, (double)o2);
            OLED_PrintASCIIString(0, 18, (char *)buf, &afont8x6, OLED_COLOR_NORMAL);
            snprintf((char *)buf, sizeof(buf), "S3(CAN2) %5.2f %5.2f %u  ", (double)c3->nox_ppm, (double)c3->o2_pct, (unsigned)c3->state);
            OLED_PrintASCIIString(0, 26, (char *)buf, &afont8x6, OLED_COLOR_NORMAL);
            snprintf((char *)buf, sizeof(buf), "Out NOx %5.2f O2 %5.2f    ", (double)NOx_ppm, (double)O2_pct);
            OLED_PrintASCIIString(0, 34, (char *)buf, &afont8x6, OLED_COLOR_NORMAL);
            snprintf((char *)buf, sizeof(buf), "state: %u                 ", (unsigned)Var_Read_OutputChStatus());
            OLED_ColorMode c = (Var_Read_OutputChStatus() == 0x1FFu) ? OLED_COLOR_NORMAL : OLED_COLOR_REVERSED;
            OLED_PrintASCIIString(0, 42, (char *)buf, &afont8x6, c);
        }

        /* Heater command: 第一路 CAN 发送；第二路 CAN（MCP2515）也发送 18FEDF55（文档 4.2 节）. */
        J1939_CAN_Transmit(&TxMessage);
        {
            MCP2515_CAN_Frame_t heater;
            heater.id = 0x18FEDF55u;
            heater.is_ext_id = true;
            heater.len = 8;
            for (int i = 0; i < 8; i++)
                heater.data[i] = HeaterData[i];
            MCP2515_Send(&heater);
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

/* ----- Default task: calibration, alarm, blowback, run time, OLED refresh ----- */
void NOxDefault(void *argument)
{
    char buf[32];
    Blowback_Init();

    for (;;) {
        /* Hold mutex across Var_Read_* here; same lock order as elsewhere to avoid deadlock. */
        LOCK_VAR();
        Calibration_NOx(&NOx_parameter, &NOx_parameter1);
        Calibration_O2(&O2_parameter, &O2_parameter1);
        UNLOCK_VAR();

        Alarm_Update();
        Blowback_Update();

        snprintf(buf, sizeof(buf), "Time:    %lus", (unsigned long)time_1s);
        OLED_PrintASCIIString(0, 1, buf, &afont8x6, OLED_COLOR_NORMAL);
        OLED_ShowFrame();

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/* ----- Modbus slave task: UART init, register/flash init, calibration init, poll ----- */
void ModBusSlave(void *argument)
{
    BLOW_CONTROL(0, 0);
    BLOW_CONTROL(1, 0);
    NoxChannel_Init();
    /* 第二路 CAN（MCP2515）：250 kbps，仅接收 SA 0x52（18F00F52）用于通道 2 */
    if (MCP2515_Init(MCP2515_BAUD_250K) == 0)
        MCP2515_SetFilter(0x18F00F52u, 0x1FFFFFFFu, true);
    MDSUARTx.Init.BaudRate = (uint32_t)SBAUD485;
    HAL_UART_Init(&MDSUARTx);
    MODRx_SemaphoreHandle = xSemaphoreCreateBinary();
    Register_Init();
#if FACTORY_FLASH_PROGRAM_ON_BOOT
    (void)FactoryFlash_ProgramDefaults();
#else
    LoadRegistersFromFlash();
#endif
    AfterFlash_Init();
    Calibration_Init();
    Start_Receive();

    for (;;)
        MODS_Poll();
}

/* ----- Register init: sync g_tVar from channel params and blowback config ----- */
void Register_Init(void)
{
    Var_Write_AlarmNoxHi((float)NOx_High);
    Var_Write_AlarmO2Lo((float)O2_Low);
    for (uint8_t ch = 0; ch < 2u; ch++) {
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
        Var_Write_SensorBlowInterval(ch, (uint16_t)(ch == 0 ? Blowback_GetInterval() : Blowback_GetIntervalCh1()));
        Var_Write_SensorBlowDuration(ch, (uint16_t)(ch == 0 ? Blowback_GetDuration() : Blowback_GetDurationCh1()));
    }
    LOCK_VAR();
    g_tVar.S1.blow_status = 0u;
    g_tVar.S1.blow_countdown = (uint16_t)(Blowback_GetInterval() > 0u ? Blowback_GetInterval() : 0u);
    g_tVar.S2.blow_status = 0u;
    g_tVar.S2.blow_countdown = (uint16_t)(Blowback_GetIntervalCh1() > 0u ? Blowback_GetIntervalCh1() : 0u);
    g_tVar.S1.power_on = 1u; /* default on; GPIO control when SENSOR_POWER_GPIO_ENABLE */
    g_tVar.S2.power_on = 1u;
    g_tVar.work_mode = 1u;   /* default: mode1 主从 ch0 主 */
    UNLOCK_VAR();
}

void AfterFlash_Init(void)
{
    for (uint8_t ch = 0; ch < 2u; ch++) {
        NoxChannel_t *c = &g_noxChannels[ch];
        c->nox_y[1] = Var_Read_SensorP2Nox(ch);
        c->o2_y[1]  = Var_Read_SensorP2O2(ch);
        c->nox_y[2] = Var_Read_SensorP3Nox(ch);
        c->o2_y[2]  = Var_Read_SensorP3O2(ch);
        c->nox_low.a  = Var_Read_SensorSeg1NoxA(ch);
        c->nox_low.b  = Var_Read_SensorSeg1NoxB(ch);
        c->o2_low.a   = Var_Read_SensorSeg1O2A(ch);
        c->o2_low.b   = Var_Read_SensorSeg1O2B(ch);
        c->nox_high.a = Var_Read_SensorSeg2NoxA(ch);
        c->nox_high.b = Var_Read_SensorSeg2NoxB(ch);
        c->o2_high.a  = Var_Read_SensorSeg2O2A(ch);
        c->o2_high.b  = Var_Read_SensorSeg2O2B(ch);
    }

    for (uint8_t ch = 0; ch < NOX_SENSOR_COUNT; ch++) {
        NoxChannel_t *c = &g_noxChannels[ch];
        NoxSensor_CalibrationInit(c->nox_x, c->nox_y, &c->nox_low, &c->nox_high);
        NoxSensor_CalibrationInit(c->o2_x,  c->o2_y,  &c->o2_low,  &c->o2_high);
    }

    Blowback_SetConfig((uint32_t)Var_Read_SensorBlowInterval(0), (uint32_t)Var_Read_SensorBlowDuration(0));
    Blowback_SetConfigCh1((uint32_t)Var_Read_SensorBlowInterval(1), (uint32_t)Var_Read_SensorBlowDuration(1));
}
