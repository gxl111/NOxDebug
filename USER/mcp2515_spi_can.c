/**
 ******************************************************************************
 * @file    mcp2515_spi_can.c
 * @brief   MCP2515 SPI 转 CAN 驱动实现（SPI2 + PC10 CS, PC11 INT）
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
#define MCP2515_REQOP_NORMAL 0x00
#define MCP2515_OPMODE_MASK  0xE0
#define MCP2515_TXB0_READY   0x00
#define MCP2515_TXREQ        0x08
#define MCP2515_TXBnCTRL_TXREQ (1u<<3)
#define MCP2515_RXB0CTRL_RXM_STD 0x20
#define MCP2515_RXB0CTRL_RXM_EXT 0x40
#define MCP2515_RXB0CTRL_RXM_ANY 0x60
#define MCP2515_RXB0CTRL_BUKT    0x04  /* 溢出到 RXB1 */
#define MCP2515_CANINTF_RX0IF    0x01
#define MCP2515_CANINTF_RX1IF    0x02
#define MCP2515_CANINTF_TX0IF    0x04
#define MCP2515_CANINTE_RX0IE    0x01
#define MCP2515_CANINTE_RX1IE    0x02

/* 16 MHz 晶振下常用波特率：CNF1, CNF2, CNF3 (BRP, SYNC+PHSEG1, PHSEG2) */
static const uint8_t cnf_125k[]  = { 0x03, 0xFA, 0x87 }; /* 125 kbps */
static const uint8_t cnf_250k[]  = { 0x00, 0xF4, 0x06 }; /* 250 kbps, 32 Tq */
static const uint8_t cnf_500k[]  = { 0x00, 0xFA, 0x06 }; /* 500 kbps, 16 Tq */
static const uint8_t cnf_1000k[] = { 0x00, 0xD0, 0x82 }; /* 1 Mbps */

static const uint8_t *const cnf_tables[] = {
	cnf_125k, cnf_250k, cnf_500k, cnf_1000k
};

static SPI_HandleTypeDef *s_spi = &hspi2;

/* 在模块内初始化 MCP2515 所用 GPIO：CS=PC10 输出，INT=PC11 输入 */
static void mcp2515_gpio_init(void)
{
	__HAL_RCC_GPIOC_CLK_ENABLE();
	GPIO_InitTypeDef g = {0};
	g.Pin   = MCP2515_CS_Pin;
	g.Mode  = GPIO_MODE_OUTPUT_PP;
	g.Pull  = GPIO_NOPULL;
	g.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(MCP2515_CS_GPIO_Port, &g);
	HAL_GPIO_WritePin(MCP2515_CS_GPIO_Port, MCP2515_CS_Pin, GPIO_PIN_SET);

	g.Pin  = MCP2515_INT_Pin;
	g.Mode = GPIO_MODE_INPUT;
	g.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(MCP2515_INT_GPIO_Port, &g);
}

#define CS_LOW()   HAL_GPIO_WritePin(MCP2515_CS_GPIO_Port, MCP2515_CS_Pin, GPIO_PIN_RESET)
#define CS_HIGH()  HAL_GPIO_WritePin(MCP2515_CS_GPIO_Port, MCP2515_CS_Pin, GPIO_PIN_SET)

static uint8_t spi_xfer_byte(uint8_t tx)
{
	uint8_t rx;
	HAL_SPI_TransmitReceive(s_spi, &tx, &rx, 1, 50);
	return rx;
}

static void spi_write_bytes(const uint8_t *buf, uint16_t len)
{
	HAL_SPI_Transmit(s_spi, (uint8_t*)buf, len, 100);
}

static void spi_read_bytes(uint8_t *buf, uint16_t len)
{
	HAL_SPI_Receive(s_spi, buf, len, 100);
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

int MCP2515_Init(MCP2515_Baud_t baud)
{
	if (baud >= MCP2515_BAUD_COUNT)
		return -1;

	mcp2515_gpio_init();
	mcp2515_reset();

	if (mcp2515_enter_config() != 0)
		return -2;

	const uint8_t *cnf = cnf_tables[baud];
	mcp2515_write_reg(MCP2515_CNF1, cnf[0]);
	mcp2515_write_reg(MCP2515_CNF2, cnf[1]);
	mcp2515_write_reg(MCP2515_CNF3, cnf[2]);

	/* 接收：RXB0 接受任意，溢出到 RXB1；使能 RX0/RX1 中断 */
	mcp2515_write_reg(MCP2515_RXB0CTRL, MCP2515_RXB0CTRL_RXM_ANY | MCP2515_RXB0CTRL_BUKT);
	mcp2515_write_reg(MCP2515_RXB1CTRL, MCP2515_RXB0CTRL_RXM_ANY);
	mcp2515_write_reg(MCP2515_CANINTE, MCP2515_CANINTE_RX0IE | MCP2515_CANINTE_RX1IE);
	mcp2515_write_reg(MCP2515_CANINTF, 0x00);

	mcp2515_set_normal();
	HAL_Delay(1);
	return 0;
}

int MCP2515_Send(const MCP2515_CAN_Frame_t *frame)
{
	if (frame->len > 8)
		return -1;

	/* 等待 TXB0 空闲 */
	uint32_t t = 0;
	while (mcp2515_read_reg(MCP2515_TXB0CTRL) & MCP2515_TXBnCTRL_TXREQ) {
		HAL_Delay(1);
		if (++t > 50)
			return -2;
	}

	uint8_t sidh, sidl, eid8, eid0;
	if (frame->is_ext_id) {
		/* 29-bit: SIDH=id[28:21], SIDL[7:5]=id[20:18], SIDL[3]=EXIDE, SIDL[1:0]=id[17:16], EID8=id[15:8], EID0=id[7:0] */
		sidh = (uint8_t)(frame->id >> 21);
		sidl = (uint8_t)(((frame->id >> 18) & 7) << 5) | 0x08 | (uint8_t)((frame->id >> 16) & 3);
		eid8 = (uint8_t)(frame->id >> 8);
		eid0 = (uint8_t)(frame->id & 0xFF);
	} else {
		sidh = (uint8_t)(frame->id >> 3);
		sidl = (uint8_t)((frame->id & 7) << 5);
		eid8 = 0;
		eid0 = 0;
	}

	uint8_t buf[13];
	buf[0] = sidh;
	buf[1] = sidl;
	buf[2] = eid8;
	buf[3] = eid0;
	buf[4] = frame->len & 0x0F;
	memcpy(&buf[5], frame->data, frame->len);
	mcp2515_write_regs(MCP2515_TXB0SIDH, buf, 5 + 8);

	/* 请求发送 */
	CS_LOW();
	spi_xfer_byte(MCP2515_INS_RTS | (1 << 0));
	CS_HIGH();
	return 0;
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

void MCP2515_SetFilter(uint32_t id, uint32_t mask, bool ext)
{
	if (mcp2515_enter_config() != 0)
		return;

	uint8_t sidh = (uint8_t)(id >> 3);
	uint8_t sidl = (uint8_t)((id & 7) << 5);
	uint8_t eid8 = (uint8_t)(id >> 16);
	uint8_t eid0 = (uint8_t)(id >> 8);
	if (ext)
		sidl |= 0x08;
	uint8_t msk_sidh = (uint8_t)(mask >> 3);
	uint8_t msk_sidl = (uint8_t)((mask & 7) << 5);
	if (ext)
		msk_sidl |= 0x08;
	uint8_t msk_eid8 = (uint8_t)(mask >> 16);
	uint8_t msk_eid0 = (uint8_t)(mask >> 8);

	mcp2515_write_reg(MCP2515_RXF0SIDH, sidh);
	mcp2515_write_reg(MCP2515_RXF0SIDL, sidl);
	mcp2515_write_reg(MCP2515_RXF0EID8, eid8);
	mcp2515_write_reg(MCP2515_RXF0EID0, eid0);
	mcp2515_write_reg(MCP2515_RXM0SIDH, msk_sidh);
	mcp2515_write_reg(MCP2515_RXM0SIDL, msk_sidl);
	mcp2515_write_reg(MCP2515_RXM0EID8, msk_eid8);
	mcp2515_write_reg(MCP2515_RXM0EID0, msk_eid0);

	mcp2515_set_normal();
}
