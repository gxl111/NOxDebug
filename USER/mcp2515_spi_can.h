/**
 ******************************************************************************
 * @file    mcp2515_spi_can.h
 * @brief   MCP2515 SPI 转 CAN 驱动头文件
 *          主控引脚：SPI2 (PB13 SCK, PB14 MISO, PB15 MOSI), CS=PC10, INT=PC11
 *          用于在 SPI-CAN 控制器上外接传感器等设备。
 ******************************************************************************
 */
#ifndef __MCP2515_SPI_CAN_H__
#define __MCP2515_SPI_CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* ----------------------------------------------------------------------------
 * 波特率枚举（16 MHz 晶振）
 * ---------------------------------------------------------------------------- */
typedef enum {
	MCP2515_BAUD_125K = 0,
	MCP2515_BAUD_250K,
	MCP2515_BAUD_500K,
	MCP2515_BAUD_1000K,
	MCP2515_BAUD_COUNT
} MCP2515_Baud_t;

/* ----------------------------------------------------------------------------
 * CAN 帧结构（标准/扩展 ID，最多 8 字节数据）
 * ---------------------------------------------------------------------------- */
typedef struct {
	uint32_t id;           /* 标准 11 位或扩展 29 位 ID */
	uint8_t  data[8];
	uint8_t  len;          /* 0..8 */
	bool     is_ext_id;    /* true = 29 位扩展 ID */
} MCP2515_CAN_Frame_t;

/* ----------------------------------------------------------------------------
 * API
 * ---------------------------------------------------------------------------- */

/**
 * @brief 初始化 MCP2515（SPI2 + CS/INT），配置波特率并进入正常模式
 * @return 0 成功；-1 baud 非法；-2 进配置模式失败；-3 未能确认 Normal 模式
 */
int MCP2515_Init(MCP2515_Baud_t baud);

/**
 * @brief 发送一帧 CAN 报文
 * @return 0 成功；
 *         -1 参数非法；
 *         -2 发送前等待 TXB0 空闲超时；
 *         -3 ID 超出范围；
 *         -4 请求发送后长时间未结束；
 *         -5 发送失败/中止；
 *         -6 仲裁丢失；
 *         -7 发送错误（常见于 ACK/位时序/物理层问题）；
 *         -8 未看到发送完成标志
 */
int MCP2515_Send(const MCP2515_CAN_Frame_t *frame);

/**
 * @brief 轮询接收一帧（无阻塞，无数据时立即返回）
 * @param frame 输出帧
 * @return 1 读到一帧，0 无数据，-1 参数非法
 */
int MCP2515_Receive(MCP2515_CAN_Frame_t *frame);

/**
 * @brief 是否已成功初始化（Init 返回 0 后为 true；未初始化或失败时为 false）
 */
bool MCP2515_IsReady(void);

/**
 * @brief INT 为低表示 MCP2515 有未处理中断（与 CANINTE 一致时多为 RX）
 */
bool MCP2515_HasInterrupt(void);

/**
 * @brief 设置接收滤波（须在 Init 成功后调用）
 * @return 0 成功；-1 无法进入配置模式；-2 未能确认进入 Normal 模式
 */
int MCP2515_SetFilter(uint32_t id, uint32_t mask, bool ext);

/**
 * @brief 调试读取关键寄存器（可用于排查 SPI/时序/模式）
 * @param canstat  CANSTAT(0x0E)
 * @param canctrl  CANCTRL(0x0F)
 * @param cnf1     CNF1(0x2A)
 * @param cnf2     CNF2(0x29)
 * @param cnf3     CNF3(0x28)
 * @param canintf  CANINTF(0x2C)
 * @param eflg     EFLG(0x2D)
 * @param tec      TEC(0x1C)
 * @param rec      REC(0x1D)
 * @return 0 成功；-1 参数为空
 */
int MCP2515_DebugReadCore(uint8_t *canstat, uint8_t *canctrl, uint8_t *cnf1, uint8_t *cnf2,
			  uint8_t *cnf3, uint8_t *canintf, uint8_t *eflg, uint8_t *tec, uint8_t *rec);

/**
 * @brief 回环自检：切到 Loopback，发一帧并尝试收回，再恢复 Normal
 * @return 0 成功；负值失败（见实现注释）
 */
int MCP2515_LoopbackSelfTest(void);

#ifdef __cplusplus
}
#endif

#endif /* __MCP2515_SPI_CAN_H__ */
