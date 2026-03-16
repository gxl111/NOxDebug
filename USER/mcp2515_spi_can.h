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
 * @param baud 波特率枚举
 * @return 0 成功，负值失败
 */
int MCP2515_Init(MCP2515_Baud_t baud);

/**
 * @brief 发送一帧 CAN 报文
 * @param frame 帧内容（id, data, len, is_ext_id）
 * @return 0 成功，负值失败
 */
int MCP2515_Send(const MCP2515_CAN_Frame_t *frame);

/**
 * @brief 轮询接收一帧（无阻塞，无数据时立即返回）
 * @param frame 输出帧
 * @return 1 读到一帧，0 无数据，负值错误
 */
int MCP2515_Receive(MCP2515_CAN_Frame_t *frame);

/**
 * @brief 是否有接收中断（INT 引脚为低表示有事件）
 */
bool MCP2515_HasInterrupt(void);

/**
 * @brief 设置接收滤波（可选）：只接受指定 ID 或范围，便于接传感器
 *        默认接收所有帧；若需过滤可在 Init 后调用此函数。
 * @param id  接受的标准/扩展 ID（与 mask 配合）
 * @param mask 掩码：1 表示必须匹配 id 的该位，0 表示不关心
 * @param ext 是否扩展 ID
 */
void MCP2515_SetFilter(uint32_t id, uint32_t mask, bool ext);

#ifdef __cplusplus
}
#endif

#endif /* __MCP2515_SPI_CAN_H__ */
