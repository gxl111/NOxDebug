/**
 * @file    NOx.c
 * @brief   NOx tasks: J1939 receive/handle (per channel), strategy (single/primary_backup/fusion),
 *          default task (calibration, alarm, blowback; OLED 由 APP_USE_OLED 控制), Modbus slave init.
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
#include "app_config.h"
#if APP_USE_OLED
#include "oled.h"
#endif
#include "sdcard.h"
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

/* 第二路 CAN（MCP2515）J1939 扩展帧 29 位 ID，与 J1939.c 中 Array[0..3] 大端打包一致 */
#define NOX_MCP2515_RX_ID      0x18F00F52u
#define NOX_MCP2515_HEATER_ID  0x18FEDF55u
#define NOX_MCP2515_BOOT_DEBUG 0

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

/* Update Modbus per-channel readings: S1/S2/S3 写入 Var. */
static void NOx_UpdatePerChannelRegs(void)
{
    LOCK_VAR();
    for (uint8_t ch = 0; ch < NOX_SENSOR_COUNT; ch++) {
        NoxChannel_t *c = &g_noxChannels[ch];
        Var_Write_SensorLiveNox(ch, c->nox_ppm);
        Var_Write_SensorLiveO2(ch, c->o2_pct);
        Var_Write_SensorStatus(ch, c->state);
    }
    UNLOCK_VAR();
}

#if APP_USE_OLED
static void NOx_OledRefresh(void)
{
    char buf[32];
    float n1 = Var_Read_SensorLiveNox(0), o1 = Var_Read_SensorLiveO2(0);
    float n2 = Var_Read_SensorLiveNox(1), o2 = Var_Read_SensorLiveO2(1);
    float n3 = Var_Read_SensorLiveNox(2), o3 = Var_Read_SensorLiveO2(2);
    uint16_t st1 = Var_Read_SensorStatus(0);
    uint16_t st2 = Var_Read_SensorStatus(1);
    uint16_t st3 = Var_Read_SensorStatus(2);

    OLED_NewFrame();
    snprintf(buf, sizeof(buf), "Time:    %lus", (unsigned long)time_1s);
    OLED_PrintASCIIString(0, 1, buf, &afont8x6, OLED_COLOR_NORMAL);
    snprintf(buf, sizeof(buf), "S1NOx %5.2f O2 %5.2f     ", (double)n1, (double)o1);
    OLED_PrintASCIIString(0, 10, buf, &afont8x6, OLED_COLOR_NORMAL);
    snprintf(buf, sizeof(buf), "S2NOx %5.2f O2 %5.2f     ", (double)n2, (double)o2);
    OLED_PrintASCIIString(0, 20, buf, &afont8x6, OLED_COLOR_NORMAL);
    snprintf(buf, sizeof(buf), "S3NOx %5.2f O2 %5.2f     ", (double)n3, (double)o3);
    OLED_PrintASCIIString(0, 30, buf, &afont8x6, OLED_COLOR_NORMAL);
    snprintf(buf, sizeof(buf), "OutNOx %5.2f O2 %5.2f    ", (double)NOx_ppm, (double)O2_pct);
    OLED_PrintASCIIString(0, 40, buf, &afont8x6, OLED_COLOR_NORMAL);
    {
        uint16_t out_st = Var_Read_OutputChStatus();
        snprintf(buf, sizeof(buf), "O%03X", (unsigned)out_st);
        OLED_PrintASCIIString(0, 50, buf, &afont8x6,
                              (out_st == 0x1FFu) ? OLED_COLOR_NORMAL : OLED_COLOR_REVERSED);
        snprintf(buf, sizeof(buf), "1%03X", (unsigned)st1);
        OLED_PrintASCIIString(32, 50, buf, &afont8x6,
                              (st1 == 0x1FFu) ? OLED_COLOR_NORMAL : OLED_COLOR_REVERSED);
        snprintf(buf, sizeof(buf), "2%03X", (unsigned)st2);
        OLED_PrintASCIIString(64, 50, buf, &afont8x6,
                              (st2 == 0x1FFu) ? OLED_COLOR_NORMAL : OLED_COLOR_REVERSED);
        snprintf(buf, sizeof(buf), "3%03X", (unsigned)st3);
        OLED_PrintASCIIString(96, 50, buf, &afont8x6,
                              (st3 == 0x1FFu) ? OLED_COLOR_NORMAL : OLED_COLOR_REVERSED);
    }
    OLED_ShowFrame();
}
#endif

/* ----- NOx receive task: dequeue J1939_RX_ITEM (channel+msg), update channel, strategy, 4-20mA, heater ----- */
void NOxReceive(void *argument)
{
    J1939_RX_ITEM item;
    TxMsg_Init(&TxMessage);
    uint32_t last_mcp_heater_tick = 0u;
    uint32_t next_mcp_heater_try_tick = 0u;

    for (;;) {
        uint8_t can1_drain = 0u;
        if (xQueueReceive(Rx_QueueHandle, &item, pdMS_TO_TICKS(NOX_RECEIVE_QUEUE_WAIT_MS)) == pdPASS) {
            do {
                NOx_HandleOne(&item.msg, item.channel_index);
                can1_drain++;
            } while (can1_drain < NOX_CAN1_RX_DRAIN_LIMIT &&
                     xQueueReceive(Rx_QueueHandle, &item, 0) == pdPASS);
        }
        /* 第二路 CAN（MCP2515）接收：SA 0x52 → 通道 2（Electrical Interface 18F00F52h） */
#if NOX_USE_MCP2515
        if (MCP2515_IsReady()) {
            MCP2515_CAN_Frame_t rx;
            uint8_t drain = 0u;
            while (drain < NOX_MCP2515_RX_DRAIN_LIMIT && MCP2515_Receive(&rx) == 1) {
                drain++;
                if (rx.is_ext_id && rx.id == NOX_MCP2515_RX_ID && rx.len >= 8) {
                    J1939_MESSAGE can2_msg;
                    can2_msg.Array[0] = (j1939_uint8_t)(rx.id >> 24);
                    can2_msg.Array[1] = (j1939_uint8_t)(rx.id >> 16);
                    can2_msg.Array[2] = (j1939_uint8_t)(rx.id >> 8);
                    can2_msg.Array[3] = (j1939_uint8_t)(rx.id);
                    can2_msg.Mxe.DataLength = 8;
                    for (int i = 0; i < 8; i++)
                        can2_msg.Mxe.Data[i] = rx.data[i];
                    NOx_HandleOne(&can2_msg, 2u);
                }
            }
        }
#endif

        NoxChannel_UpdateCommTimeouts(HAL_GetTick());

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
            Var_Write_MaNox((uint16_t)(((uint16_t)electricity_data_buf[0] << 8) | electricity_data_buf[1]));
            Var_Write_MaO2((uint16_t)(((uint16_t)electricity_data_buf[2] << 8) | electricity_data_buf[3]));
            for (uint8_t ch = 0; ch < NOX_SENSOR_COUNT; ch++) {
                NoxChannel_t *c = &g_noxChannels[ch];
                uint8_t *p = &electricity_data_buf_6ch[ch * 4u];
                NoxSensor_To4_20mA(c->nox_ppm, c->o2_pct, p);
                Var_Write_MaSensorNox(ch, (uint16_t)(((uint16_t)p[0] << 8) | p[1]));
                Var_Write_MaSensorO2(ch, (uint16_t)(((uint16_t)p[2] << 8) | p[3]));
            }
        }

#if 0
        {
            uint8_t buf[28];
            float n1 = Var_Read_SensorLiveNox(0), o1 = Var_Read_SensorLiveO2(0);
            float n2 = Var_Read_SensorLiveNox(1), o2 = Var_Read_SensorLiveO2(1);
            float n3 = Var_Read_SensorLiveNox(2), o3 = Var_Read_SensorLiveO2(2);
            uint16_t st3 = Var_Read_SensorStatus(2);
            snprintf((char *)buf, sizeof(buf), "S1 NOx %5.2f O2 %5.2f     ", (double)n1, (double)o1);
            OLED_PrintASCIIString(0, 10, (char *)buf, &afont8x6, OLED_COLOR_NORMAL);
            snprintf((char *)buf, sizeof(buf), "S2 NOx %5.2f O2 %5.2f     ", (double)n2, (double)o2);
            OLED_PrintASCIIString(0, 20, (char *)buf, &afont8x6, OLED_COLOR_NORMAL);
            snprintf((char *)buf, sizeof(buf), "S3 NOx %5.2f O2 %5.2f     ", (double)n3, (double)o3);
            OLED_PrintASCIIString(0, 30, (char *)buf, &afont8x6, OLED_COLOR_NORMAL);
            snprintf((char *)buf, sizeof(buf), "Out NOx %5.2f O2 %5.2f    ", (double)NOx_ppm, (double)O2_pct);
            OLED_PrintASCIIString(0, 40, (char *)buf, &afont8x6, OLED_COLOR_NORMAL);
            snprintf((char *)buf, sizeof(buf), "state: %u S3st: %u         ", (unsigned)Var_Read_OutputChStatus(), (unsigned)st3);
            OLED_ColorMode c = (Var_Read_OutputChStatus() == 0x1FFu) ? OLED_COLOR_NORMAL : OLED_COLOR_REVERSED;
            OLED_PrintASCIIString(0, 50, (char *)buf, &afont8x6, c);
        }
#endif

        /* Heater command: 第一路 CAN 发送；第二路 CAN（MCP2515）也发送 18FEDF55（文档 4.2 节）. */
        J1939_CAN_Transmit(&TxMessage);
#if NOX_USE_MCP2515
        if (MCP2515_IsReady()) {
            uint32_t now = HAL_GetTick();
            if ((int32_t)(now - next_mcp_heater_try_tick) >= 0 &&
                (now - last_mcp_heater_tick) >= NOX_MCP2515_HEATER_PERIOD_MS) {
                MCP2515_CAN_Frame_t heater;
                heater.id = NOX_MCP2515_HEATER_ID;
                heater.is_ext_id = true;
                heater.len = 8;
                for (int i = 0; i < 8; i++)
                    heater.data[i] = HeaterData[i];
                last_mcp_heater_tick = now;
                if (MCP2515_Send(&heater) != 0)
                    next_mcp_heater_try_tick = now + NOX_MCP2515_HEATER_RETRY_MS;
            }
        }
#endif

        vTaskDelay(pdMS_TO_TICKS(NOX_RECEIVE_LOOP_DELAY_MS));
    }
}

/* ----- Default task: calibration, alarm, blowback, run time; OLED 见 APP_USE_OLED ----- */
void NOxDefault(void *argument)
{
    Blowback_Init();

    for (;;) {
        LOCK_VAR();
        Calibration_NOx(&NOx_parameter, &NOx_parameter1);
        Calibration_O2(&O2_parameter, &O2_parameter1);
        UNLOCK_VAR();

        Alarm_Update();
        Blowback_Update();
        Modbus_AutoSuctionValvesUpdate();

#if APP_USE_OLED
        NOx_OledRefresh();
#endif

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/* ----- Modbus slave task: UART init, register/flash init, calibration init, poll ----- */
void ModBusSlave(void *argument)
{
    BLOW_CONTROL(0, 0);
    BLOW_CONTROL(1, 0);
    BLOW_CONTROL(2, 0);
    NoxChannel_Init();
    /* 第二路 CAN（MCP2515）：250 kbps；硬件滤波目标 PGN+F0 SA52 */
#if NOX_USE_MCP2515
    if (MCP2515_Init(MCP2515_BAUD_250K) == 0) {
#if NOX_MCP2515_BOOT_DEBUG
        uint8_t canstat = 0, canctrl = 0, cnf1 = 0, cnf2 = 0, cnf3 = 0;
        uint8_t canintf = 0, eflg = 0, tec = 0, rec = 0;
        int mcp_dbg = MCP2515_DebugReadCore(&canstat, &canctrl, &cnf1, &cnf2, &cnf3,
                                            &canintf, &eflg, &tec, &rec);
        int mcp_lb = MCP2515_LoopbackSelfTest();
        (void)mcp_dbg; (void)mcp_lb;
        (void)canstat; (void)canctrl; (void)cnf1; (void)cnf2; (void)cnf3;
        (void)canintf; (void)eflg; (void)tec; (void)rec;
#endif

        (void)MCP2515_SetFilter(NOX_MCP2515_RX_ID, 0x1FFFFFFFu, true);
    }
#endif
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
#if SENSOR_POWER_GPIO_ENABLE
    /* 将 power_on 寄存器同步到 GPIO（PC0/PC13/PB9），上电后输出与寄存器一致 */
    for (uint8_t ch = 0; ch < NOX_SENSOR_COUNT; ch++)
        Var_Write_SensorPowerOn(ch, Var_Read_SensorPowerOn(ch));
#endif
    Start_Receive();

    for (;;)
        MODS_Poll();
}

/* ----- Register init: sync g_tVar from channel params and blowback config ----- */
void Register_Init(void)
{
    Var_Write_AlarmNoxHi((float)NOx_High);
    Var_Write_AlarmO2Lo((float)O2_Low);
    for (uint8_t ch = 0; ch < NOX_SENSOR_COUNT; ch++) {
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
        Var_Write_SensorBlowInterval(ch, (uint16_t)DEFAULT_BLOW_INTERVAL);
        Var_Write_SensorBlowDuration(ch, (uint16_t)DEFAULT_BLOW_DURATION);
    }
    LOCK_VAR();
    g_tVar.S1.blow_status = 0u;
    g_tVar.S1.blow_countdown = (uint16_t)DEFAULT_BLOW_INTERVAL;
    g_tVar.S2.blow_status = 0u;
    g_tVar.S2.blow_countdown = (uint16_t)(DEFAULT_BLOW_INTERVAL / BLOW_PHASE_DIVISOR);
    g_tVar.S3.blow_status = 0u;
    g_tVar.S3.blow_countdown = (uint16_t)((2u * DEFAULT_BLOW_INTERVAL) / BLOW_PHASE_DIVISOR);
    g_tVar.S1.power_on = 1u;
    g_tVar.S2.power_on = 1u;
    g_tVar.S3.power_on = 1u;
    g_tVar.work_mode = 1u;   /* default: mode1 主从 ch0 主 */
    UNLOCK_VAR();
}

void AfterFlash_Init(void)
{
    for (uint8_t ch = 0; ch < NOX_SENSOR_COUNT; ch++) {
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

    {
        uint16_t shared_interval = Var_Read_SensorBlowInterval(0);
        if (shared_interval == 0u || shared_interval == 0xFFFFu)
            shared_interval = (uint16_t)DEFAULT_BLOW_INTERVAL;
        for (uint8_t ch = 0; ch < NOX_SENSOR_COUNT; ch++) {
            uint16_t duration = Var_Read_SensorBlowDuration(ch);
            Var_Write_SensorBlowInterval(ch, shared_interval);
            if (duration == 0u || duration == 0xFFFFu)
                Var_Write_SensorBlowDuration(ch, (uint16_t)DEFAULT_BLOW_DURATION);
        }
    }

    Blowback_SetConfig((uint32_t)Var_Read_SensorBlowInterval(0), (uint32_t)Var_Read_SensorBlowDuration(0));
    Blowback_SetConfigCh1((uint32_t)Var_Read_SensorBlowInterval(1), (uint32_t)Var_Read_SensorBlowDuration(1));
    Blowback_SetConfigCh2((uint32_t)Var_Read_SensorBlowInterval(2), (uint32_t)Var_Read_SensorBlowDuration(2));
}
