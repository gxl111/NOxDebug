/**
 ******************************************************************************
 * @file    mcp2515_spi_can.c
 * @brief   MCP2515 SPI 转 CAN 驱动实现（SPI2 + CS/INT 由 CubeMX gpio.c 配置）
 ******************************************************************************
 */
#include "mcp2515_spi_can.h"
#include "main.h"
#include "spi.h"
#include <string.h>

/* ----------------------------------------------------------------------------
 * MCP2515 SPI 指令
 * ---------------------------------------------------------------------------- */
#define MCP2515_INS_RESET       0xC0
#define MCP2515_INS_READ        0x03
#define MCP2515_INS_WRITE       0x02
#define MCP2515_INS_RTS         0x80   /* RTS | (1<<n) for TXBn */
#define MCP2515_INS_READ_STATUS 0xA0
#define MCP2515_INS_RX_STATUS   0xB0
#define MCP2515_INS_BIT_MODIFY   0x05

/* ----------------------------------------------------------------------------
 * 寄存器地址（MCP2515 数据手册）
 * ---------------------------------------------------------------------------- */
#define MCP2515_CANCTRL  0x0F
#define MCP2515_CANSTAT  0x0E
#define MCP2515_CNF1     0x2A
#define MCP2515_CNF2     0x29
#define MCP2515_CNF3     0x28
#define MCP2515_CANINTE  0x2B
#define MCP2515_CANINTF  0x2C
#define MCP2515_EFLG     0x2D
#define MCP2515_TEC      0x1C
#define MCP2515_REC      0x1D
/* TXB0 */
#define MCP2515_TXB0CTRL 0x30
#define MCP2515_TXB0SIDH 0x31
#define MCP2515_TXB0SIDL 0x32
#define MCP2515_TXB0EID8 0x33
#define MCP2515_TXB0EID0 0x34
#define MCP2515_TXB0DLC  0x35
#define MCP2515_TXB0D0   0x36
/* RXB0 */
#define MCP2515_RXB0CTRL 0x60
#define MCP2515_RXB0SIDH 0x61
#define MCP2515_RXB0SIDL 0x62
#define MCP2515_RXB0EID8 0x63
#define MCP2515_RXB0EID0 0x64
#define MCP2515_RXB0DLC  0x65
#define MCP2515_RXB0D0   0x66
/* RXB1 */
#define MCP2515_RXB1CTRL 0x70
#define MCP2515_RXB1SIDH 0x71
#define MCP2515_RXB1SIDL 0x72
#define MCP2515_RXB1EID8 0x73
#define MCP2515_RXB1EID0 0x74
#define MCP2515_RXB1DLC  0x75
#define MCP2515_RXB1D0   0x76
/* 验收滤波 */
#define MCP2515_RXF0SIDH 0x00
#define MCP2515_RXF0SIDL 0x01
#define MCP2515_RXF0EID8 0x02
#define MCP2515_RXF0EID0 0x03
#define MCP2515_RXM0SIDH 0x20
#define MCP2515_RXM0SIDL 0x21
#define MCP2515_RXM0EID8 0x22
#define MCP2515_RXM0EID0 0x23

#define MCP2515_REQOP_CONFIG 0x80
#define MCP2515_REQOP_LOOPBACK 0x40
#define MCP2515_REQOP_NORMAL 0x00
#define MCP2515_OPMODE_MASK  0xE0
#define MCP2515_CANCTRL_ABAT 0x10
#define MCP2515_CANCTRL_OSM  0x08
#define MCP2515_TXB0_READY   0x00
#define MCP2515_TXREQ        0x08
#define MCP2515_TXBnCTRL_TXREQ (1u<<3)
#define MCP2515_TXBnCTRL_TXERR (1u<<4)
#define MCP2515_TXBnCTRL_MLOA  (1u<<5)
#define MCP2515_TXBnCTRL_ABTF  (1u<<6)
#define MCP2515_RXB0CTRL_RXM_STD 0x20
#define MCP2515_RXB0CTRL_RXM_EXT 0x40
#define MCP2515_RXB0CTRL_RXM_ANY 0x60
#define MCP2515_RXB0CTRL_BUKT    0x04  /* 溢出到 RXB1 */
#define MCP2515_CANINTF_RX0IF    0x01
#define MCP2515_CANINTF_RX1IF    0x02
#define MCP2515_CANINTF_TX0IF    0x04
#define MCP2515_CANINTE_RX0IE    0x01
#define MCP2515_CANINTE_RX1IE    0x02

/* 板级原理图为 MCP2515 外挂 8 MHz 晶振。
 * 说明：现网仅使用 250 kbps（J1939）。此处将 250k 映射到经验证可工作的
 * 寄存器组（原 16MHz 表中的 500k 组，在 8MHz 下等效 250k）。
 */
static const uint8_t cnf_125k[]  = { 0x03, 0xFA, 0x87 }; /* 125 kbps */
static const uint8_t cnf_250k[]  = { 0x00, 0xFA, 0x06 }; /* 8MHz 晶振下等效 250 kbps */
static const uint8_t cnf_500k[]  = { 0x00, 0xFA, 0x06 }; /* 500 kbps, 16 Tq */
static const uint8_t cnf_1000k[] = { 0x00, 0xD0, 0x82 }; /* 1 Mbps */

static const uint8_t *const cnf_tables[] = {
	cnf_125k, cnf_250k, cnf_500k, cnf_1000k
};

static SPI_HandleTypeDef *s_spi = &hspi2;
static volatile bool s_mcp2515_ready = false;
static uint8_t s_last_txb0ctrl = 0;
static uint8_t s_last_canintf = 0;
static uint8_t s_last_eflg = 0;
static uint8_t s_last_tec = 0;
static uint8_t s_last_rec = 0;

#define CS_LOW()   HAL_GPIO_WritePin(MCP2515_CS_GPIO_Port, MCP2515_CS_Pin, GPIO_PIN_RESET)
#define CS_HIGH()  HAL_GPIO_WritePin(MCP2515_CS_GPIO_Port, MCP2515_CS_Pin, GPIO_PIN_SET)

static uint8_t spi_xfer_byte(uint8_t tx)
{
	uint8_t rx = 0xFFu;
	if (HAL_SPI_TransmitReceive(s_spi, &tx, &rx, 1, 50) != HAL_OK)
		return 0xFFu;
	return rx;
}

/* 与 TX/RX 寄存器布局一致：标准 11 位或扩展 29 位 */
static void mcp2515_id_to_regs(uint32_t id, bool ext,
			       uint8_t *sidh, uint8_t *sidl, uint8_t *eid8, uint8_t *eid0)
{
	if (ext) {
		*sidh = (uint8_t)(id >> 21);
		*sidl = (uint8_t)(((id >> 18) & 7u) << 5) | 0x08u
			| (uint8_t)((id >> 16) & 3u);
		*eid8 = (uint8_t)(id >> 8);
		*eid0 = (uint8_t)(id & 0xFFu);
	} else {
		id &= 0x7FFu;
		*sidh = (uint8_t)(id >> 3);
		*sidl = (uint8_t)((id & 7u) << 5);
		*eid8 = 0;
		*eid0 = 0;
	}
}

static uint8_t mcp2515_read_reg(uint8_t addr)
{
	CS_LOW();
	spi_xfer_byte(MCP2515_INS_READ);
	spi_xfer_byte(addr);
	uint8_t v = spi_xfer_byte(0);
	CS_HIGH();
	return v;
}

static int mcp2515_verify_opmode(uint8_t expected_mode_bits)
{
	uint8_t stat = mcp2515_read_reg(MCP2515_CANSTAT);
	if ((stat & MCP2515_OPMODE_MASK) != expected_mode_bits)
		return -1;
	return 0;
}

static void mcp2515_write_reg(uint8_t addr, uint8_t val)
{
	CS_LOW();
	spi_xfer_byte(MCP2515_INS_WRITE);
	spi_xfer_byte(addr);
	spi_xfer_byte(val);
	CS_HIGH();
}

static void mcp2515_read_regs(uint8_t addr, uint8_t *buf, uint8_t n)
{
	CS_LOW();
	spi_xfer_byte(MCP2515_INS_READ);
	spi_xfer_byte(addr);
	for (uint8_t i = 0; i < n; i++)
		buf[i] = spi_xfer_byte(0);
	CS_HIGH();
}

static void mcp2515_write_regs(uint8_t addr, const uint8_t *buf, uint8_t n)
{
	CS_LOW();
	spi_xfer_byte(MCP2515_INS_WRITE);
	spi_xfer_byte(addr);
	for (uint8_t i = 0; i < n; i++)
		spi_xfer_byte(buf[i]);
	CS_HIGH();
}

static void mcp2515_bit_modify(uint8_t addr, uint8_t mask, uint8_t val)
{
	CS_LOW();
	spi_xfer_byte(MCP2515_INS_BIT_MODIFY);
	spi_xfer_byte(addr);
	spi_xfer_byte(mask);
	spi_xfer_byte(val);
	CS_HIGH();
}

static void mcp2515_reset(void)
{
	CS_LOW();
	spi_xfer_byte(MCP2515_INS_RESET);
	CS_HIGH();
	HAL_Delay(1);
}

static int mcp2515_enter_config(void)
{
	mcp2515_bit_modify(MCP2515_CANCTRL, MCP2515_OPMODE_MASK, MCP2515_REQOP_CONFIG);
	HAL_Delay(1);
	uint8_t stat = mcp2515_read_reg(MCP2515_CANSTAT);
	if ((stat & MCP2515_OPMODE_MASK) != MCP2515_REQOP_CONFIG)
		return -1;
	return 0;
}

static void mcp2515_set_normal(void)
{
	mcp2515_bit_modify(MCP2515_CANCTRL, MCP2515_OPMODE_MASK, MCP2515_REQOP_NORMAL);
}

static void mcp2515_set_one_shot(bool enable)
{
	mcp2515_bit_modify(MCP2515_CANCTRL, MCP2515_CANCTRL_OSM, enable ? MCP2515_CANCTRL_OSM : 0u);
}

static void mcp2515_abort_all_tx(void)
{
	mcp2515_bit_modify(MCP2515_CANCTRL, MCP2515_CANCTRL_ABAT, MCP2515_CANCTRL_ABAT);
	HAL_Delay(1);
	mcp2515_bit_modify(MCP2515_CANCTRL, MCP2515_CANCTRL_ABAT, 0u);
}

static void mcp2515_capture_tx_diag(void)
{
	s_last_txb0ctrl = mcp2515_read_reg(MCP2515_TXB0CTRL);
	s_last_canintf = mcp2515_read_reg(MCP2515_CANINTF);
	s_last_eflg = mcp2515_read_reg(MCP2515_EFLG);
	s_last_tec = mcp2515_read_reg(MCP2515_TEC);
	s_last_rec = mcp2515_read_reg(MCP2515_REC);
}

int MCP2515_Init(MCP2515_Baud_t baud)
{
	if (baud >= MCP2515_BAUD_COUNT)
		return -1;

	s_mcp2515_ready = false;

	/* 依赖 main 中先于本函数调用 MX_GPIO_Init（CS/INT）与 MX_SPI2_Init */
	mcp2515_reset();

	if (mcp2515_enter_config() != 0)
		return -2;

	const uint8_t *cnf = cnf_tables[baud];
	mcp2515_write_reg(MCP2515_CNF1, cnf[0]);
	mcp2515_write_reg(MCP2515_CNF2, cnf[1]);
	mcp2515_write_reg(MCP2515_CNF3, cnf[2]);
	mcp2515_set_one_shot(true);

	/* 接收：RXB0 接受任意，溢出到 RXB1；使能 RX0/RX1 中断 */
	mcp2515_write_reg(MCP2515_RXB0CTRL, MCP2515_RXB0CTRL_RXM_ANY | MCP2515_RXB0CTRL_BUKT);
	mcp2515_write_reg(MCP2515_RXB1CTRL, MCP2515_RXB0CTRL_RXM_ANY);
	mcp2515_write_reg(MCP2515_CANINTE, MCP2515_CANINTE_RX0IE | MCP2515_CANINTE_RX1IE);
	mcp2515_write_reg(MCP2515_CANINTF, 0x00);

	mcp2515_set_normal();
	HAL_Delay(1);
	if (mcp2515_verify_opmode(MCP2515_REQOP_NORMAL) != 0)
		return -3;

	s_mcp2515_ready = true;
	return 0;
}

bool MCP2515_IsReady(void)
{
	return s_mcp2515_ready;
}

int MCP2515_Send(const MCP2515_CAN_Frame_t *frame)
{
	uint8_t txb0ctrl;
	uint8_t canintf;

	if (frame == NULL || frame->len > 8)
		return -1;
	if (!frame->is_ext_id) {
		if (frame->id > 0x7FFu)
			return -3;
	} else if (frame->id > 0x1FFFFFFFu) {
		return -3;
	}

	/* 等待 TXB0 空闲 */
	uint32_t t = 0;
	while (mcp2515_read_reg(MCP2515_TXB0CTRL) & MCP2515_TXBnCTRL_TXREQ) {
		HAL_Delay(1);
		if (++t > 50) {
			mcp2515_abort_all_tx();
			mcp2515_capture_tx_diag();
			return -2;
		}
	}

	uint8_t sidh, sidl, eid8, eid0;
	mcp2515_id_to_regs(frame->id, frame->is_ext_id, &sidh, &sidl, &eid8, &eid0);

	uint8_t buf[13];
	memset(buf, 0, sizeof(buf));
	buf[0] = sidh;
	buf[1] = sidl;
	buf[2] = eid8;
	buf[3] = eid0;
	buf[4] = frame->len & 0x0Fu;
	memcpy(&buf[5], frame->data, frame->len);
	/* 只写 SIDH..DLC + 实际数据字节，避免 len<8 时把栈上垃圾写入 MCP2515 */
	mcp2515_write_regs(MCP2515_TXB0SIDH, buf, (uint8_t)(5u + frame->len));
	/* 清发送完成标志，便于本次发送结果判断 */
	mcp2515_bit_modify(MCP2515_CANINTF, MCP2515_CANINTF_TX0IF, 0);

	/* 请求发送 */
	CS_LOW();
	spi_xfer_byte(MCP2515_INS_RTS | (1 << 0));
	CS_HIGH();

	/* 等待本次发送结束，再判断是否真正发上总线 */
	for (t = 0; t < 20u; t++) {
		txb0ctrl = mcp2515_read_reg(MCP2515_TXB0CTRL);
		if ((txb0ctrl & MCP2515_TXBnCTRL_TXREQ) == 0u)
			break;
		HAL_Delay(1);
	}
	if (t >= 20u) {
		mcp2515_abort_all_tx();
		mcp2515_capture_tx_diag();
		return -4; /* 发送请求长时间未结束 */
	}

	mcp2515_capture_tx_diag();
	txb0ctrl = s_last_txb0ctrl;
	canintf = s_last_canintf;
	if (txb0ctrl & MCP2515_TXBnCTRL_ABTF)
		return -5; /* 发送中止/失败 */
	if (txb0ctrl & MCP2515_TXBnCTRL_MLOA)
		return -6; /* 仲裁丢失 */
	if (txb0ctrl & MCP2515_TXBnCTRL_TXERR)
		return -7; /* 发送错误，常见于 ACK/位时序/物理层问题 */
	if ((canintf & MCP2515_CANINTF_TX0IF) == 0u)
		return -8; /* 未看到发送完成标志 */

	mcp2515_bit_modify(MCP2515_CANINTF, MCP2515_CANINTF_TX0IF, 0);
	return 0;
}

void MCP2515_GetLastTxDiag(uint8_t *txb0ctrl, uint8_t *canintf, uint8_t *eflg, uint8_t *tec, uint8_t *rec)
{
	if (txb0ctrl != NULL) {
		*txb0ctrl = s_last_txb0ctrl;
	}
	if (canintf != NULL) {
		*canintf = s_last_canintf;
	}
	if (eflg != NULL) {
		*eflg = s_last_eflg;
	}
	if (tec != NULL) {
		*tec = s_last_tec;
	}
	if (rec != NULL) {
		*rec = s_last_rec;
	}
}

static void read_rxb_to_frame(uint8_t base_sidh, MCP2515_CAN_Frame_t *frame)
{
	uint8_t buf[13];
	mcp2515_read_regs(base_sidh, buf, 13);
	frame->is_ext_id = (buf[1] & 0x08) ? true : false;
	uint32_t id;
	if (frame->is_ext_id) {
		/* 29-bit: SIDH(8)+SIDL[7:5](3) = 11bit, SIDL[1:0]=EID17-16, EID8=EID15-8, EID0=EID7-0 */
		id = ((uint32_t)buf[0] << 21) | (((uint32_t)(buf[1] >> 5) & 7) << 18)
		   | (((uint32_t)(buf[1] & 3)) << 16) | ((uint32_t)buf[2] << 8) | buf[3];
	} else {
		id = ((uint32_t)buf[0] << 3) | (buf[1] >> 5);
	}
	frame->id = id;
	frame->len = buf[4] & 0x0F;
	if (frame->len > 8)
		frame->len = 8;
	memcpy(frame->data, &buf[5], frame->len);
}

int MCP2515_Receive(MCP2515_CAN_Frame_t *frame)
{
	if (frame == NULL)
		return -1;

	uint8_t intf = mcp2515_read_reg(MCP2515_CANINTF);
	if (intf & MCP2515_CANINTF_RX0IF) {
		read_rxb_to_frame(MCP2515_RXB0SIDH, frame);
		mcp2515_bit_modify(MCP2515_CANINTF, MCP2515_CANINTF_RX0IF, 0);
		return 1;
	}
	if (intf & MCP2515_CANINTF_RX1IF) {
		read_rxb_to_frame(MCP2515_RXB1SIDH, frame);
		mcp2515_bit_modify(MCP2515_CANINTF, MCP2515_CANINTF_RX1IF, 0);
		return 1;
	}
	return 0;
}

bool MCP2515_HasInterrupt(void)
{
	return HAL_GPIO_ReadPin(MCP2515_INT_GPIO_Port, MCP2515_INT_Pin) == GPIO_PIN_RESET;
}

int MCP2515_SetFilter(uint32_t id, uint32_t mask, bool ext)
{
	if (mcp2515_enter_config() != 0)
		return -1;

	uint8_t sidh, sidl, eid8, eid0;
	uint8_t msk_sidh, msk_sidl, msk_eid8, msk_eid0;
	mcp2515_id_to_regs(id, ext, &sidh, &sidl, &eid8, &eid0);
	mcp2515_id_to_regs(mask, ext, &msk_sidh, &msk_sidl, &msk_eid8, &msk_eid0);

	mcp2515_write_reg(MCP2515_RXF0SIDH, sidh);
	mcp2515_write_reg(MCP2515_RXF0SIDL, sidl);
	mcp2515_write_reg(MCP2515_RXF0EID8, eid8);
	mcp2515_write_reg(MCP2515_RXF0EID0, eid0);
	mcp2515_write_reg(MCP2515_RXM0SIDH, msk_sidh);
	mcp2515_write_reg(MCP2515_RXM0SIDL, msk_sidl);
	mcp2515_write_reg(MCP2515_RXM0EID8, msk_eid8);
	mcp2515_write_reg(MCP2515_RXM0EID0, msk_eid0);

	/* Init 里 RXM=11 会忽略滤波器；此处改为按滤波器验收（扩展/标准） */
	if (ext) {
		mcp2515_write_reg(MCP2515_RXB0CTRL, 0x40u | MCP2515_RXB0CTRL_BUKT);
		mcp2515_write_reg(MCP2515_RXB1CTRL, 0x40u);
	} else {
		mcp2515_write_reg(MCP2515_RXB0CTRL, 0x20u | MCP2515_RXB0CTRL_BUKT);
		mcp2515_write_reg(MCP2515_RXB1CTRL, 0x20u);
	}

	mcp2515_set_normal();
	HAL_Delay(1);
	if (mcp2515_verify_opmode(MCP2515_REQOP_NORMAL) != 0)
		return -2;

	return 0;
}

int MCP2515_DebugReadCore(uint8_t *canstat, uint8_t *canctrl, uint8_t *cnf1, uint8_t *cnf2,
			  uint8_t *cnf3, uint8_t *canintf, uint8_t *eflg, uint8_t *tec, uint8_t *rec)
{
	if (canstat == NULL || canctrl == NULL || cnf1 == NULL || cnf2 == NULL
		|| cnf3 == NULL || canintf == NULL || eflg == NULL || tec == NULL || rec == NULL)
		return -1;

	*canstat = mcp2515_read_reg(MCP2515_CANSTAT);
	*canctrl = mcp2515_read_reg(MCP2515_CANCTRL);
	*cnf1 = mcp2515_read_reg(MCP2515_CNF1);
	*cnf2 = mcp2515_read_reg(MCP2515_CNF2);
	*cnf3 = mcp2515_read_reg(MCP2515_CNF3);
	*canintf = mcp2515_read_reg(MCP2515_CANINTF);
	*eflg = mcp2515_read_reg(MCP2515_EFLG);
	*tec = mcp2515_read_reg(MCP2515_TEC);
	*rec = mcp2515_read_reg(MCP2515_REC);
	return 0;
}

int MCP2515_LoopbackSelfTest(void)
{
	MCP2515_CAN_Frame_t tx = {0};
	MCP2515_CAN_Frame_t rx = {0};
	uint32_t t;

	/* -1: 无法进入配置模式 */
	if (mcp2515_enter_config() != 0)
		return -1;

	/* 进入 loopback 模式（REQOP=100） */
	mcp2515_bit_modify(MCP2515_CANCTRL, MCP2515_OPMODE_MASK, MCP2515_REQOP_LOOPBACK);
	HAL_Delay(1);
	/* -2: 未能确认进入 loopback */
	if (mcp2515_verify_opmode(MCP2515_REQOP_LOOPBACK) != 0)
		return -2;

	/* 清接收中断标志，避免历史状态干扰 */
	mcp2515_bit_modify(MCP2515_CANINTF, MCP2515_CANINTF_RX0IF | MCP2515_CANINTF_RX1IF, 0);

	tx.id = 0x18FEDF55u;
	tx.is_ext_id = true;
	tx.len = 8;
	tx.data[0] = 0xA5u;
	tx.data[1] = 0x5Au;
	tx.data[2] = 0x11u;
	tx.data[3] = 0x22u;
	tx.data[4] = 0x33u;
	tx.data[5] = 0x44u;
	tx.data[6] = 0x55u;
	tx.data[7] = 0x66u;

	/* -3: 发送失败 */
	if (MCP2515_Send(&tx) != 0)
		return -3;

	/* 等待 loopback 收到帧 */
	for (t = 0; t < 20u; t++) {
		if (MCP2515_Receive(&rx) == 1)
			break;
		HAL_Delay(1);
	}
	/* -4: loopback 未收到 */
	if (t >= 20u)
		return -4;

	/* -5: 收到但内容不一致 */
	if (!rx.is_ext_id || rx.id != tx.id || rx.len != tx.len || memcmp(rx.data, tx.data, tx.len) != 0)
		return -5;

	/* 恢复 normal 模式 */
	mcp2515_set_normal();
	HAL_Delay(1);
	/* -6: 未能恢复 normal */
	if (mcp2515_verify_opmode(MCP2515_REQOP_NORMAL) != 0)
		return -6;

	return 0;
}
