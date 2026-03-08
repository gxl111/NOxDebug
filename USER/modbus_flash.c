/*
 * modbus_flash.c - Save/load sensor calibration (S1 then S2, same layout) to/from internal Flash.
 */
#include "modbus_flash.h"
#include "modbus_slave.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_flash_ex.h"

#define NUM_FLASH_FLOATS  24

static void float_to_buf(const float *src, uint32_t *dst, int n)
{
    union { float f; uint32_t u; } c;
    for (int i = 0; i < n; i++) {
        c.f = src[i];
        dst[i] = c.u;
    }
}

static void buf_to_float(const uint32_t *src, float *dst, int n)
{
    union { float f; uint32_t u; } c;
    for (int i = 0; i < n; i++) {
        c.u = src[i];
        dst[i] = c.f;
    }
}

int InternalFlash_Write(void)
{
    /* Order: S1 seg1, seg2, points (12 floats), then S2 same (12 floats). */
    float src[NUM_FLASH_FLOATS] = {
        g_tVar.S1.seg1_nox_a, g_tVar.S1.seg1_nox_b, g_tVar.S1.seg1_o2_a, g_tVar.S1.seg1_o2_b,
        g_tVar.S1.seg2_nox_a, g_tVar.S1.seg2_nox_b, g_tVar.S1.seg2_o2_a, g_tVar.S1.seg2_o2_b,
        g_tVar.S1.p2_nox, g_tVar.S1.p2_o2, g_tVar.S1.p3_nox, g_tVar.S1.p3_o2,
        g_tVar.S2.seg1_nox_a, g_tVar.S2.seg1_nox_b, g_tVar.S2.seg1_o2_a, g_tVar.S2.seg1_o2_b,
        g_tVar.S2.seg2_nox_a, g_tVar.S2.seg2_nox_b, g_tVar.S2.seg2_o2_a, g_tVar.S2.seg2_o2_b,
        g_tVar.S2.p2_nox, g_tVar.S2.p2_o2, g_tVar.S2.p3_nox, g_tVar.S2.p3_o2
    };
    uint32_t DATA_32[NUM_FLASH_FLOATS];
    float_to_buf(src, DATA_32, NUM_FLASH_FLOATS);

    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t SECTORError;

    HAL_FLASH_Unlock();

    uint32_t NbrOfPage = (FLASH_USER_END_ADDR - FLASH_USER_START_ADDR) / FLASH_PAGE_SIZE;
    EraseInitStruct.TypeErase     = FLASH_TYPEERASE_PAGES;
    EraseInitStruct.NbPages       = NbrOfPage;
    EraseInitStruct.PageAddress   = FLASH_USER_START_ADDR;

    if (HAL_FLASHEx_Erase(&EraseInitStruct, &SECTORError) != HAL_OK) {
        HAL_FLASH_Lock();
        return -1;
    }

    uint32_t Address = FLASH_USER_START_ADDR;
    for (int i = 0; i < NUM_FLASH_FLOATS && Address < FLASH_USER_END_ADDR; i++) {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, Address, DATA_32[i]) != HAL_OK) {
            HAL_FLASH_Lock();
            return -1;
        }
        Address += 4;
    }

    HAL_FLASH_Lock();

    /* Verify */
    Address = FLASH_USER_START_ADDR;
    for (int j = 0; j < NUM_FLASH_FLOATS && Address < FLASH_USER_END_ADDR; j++, Address += 4) {
        if (*(__IO uint32_t *)Address != DATA_32[j])
            return 0;
    }
    return 1;
}

void LoadRegistersFromFlash(void)
{
    uint32_t Address = FLASH_USER_START_ADDR;
    uint32_t raw[NUM_FLASH_FLOATS];
    for (int i = 0; i < NUM_FLASH_FLOATS && Address < FLASH_USER_END_ADDR; i++, Address += 4)
        raw[i] = *(__IO uint32_t *)Address;

    float dst[NUM_FLASH_FLOATS];
    buf_to_float(raw, dst, NUM_FLASH_FLOATS);

    g_tVar.S1.seg1_nox_a = dst[0];  g_tVar.S1.seg1_nox_b = dst[1];  g_tVar.S1.seg1_o2_a = dst[2];  g_tVar.S1.seg1_o2_b = dst[3];
    g_tVar.S1.seg2_nox_a = dst[4];  g_tVar.S1.seg2_nox_b = dst[5];  g_tVar.S1.seg2_o2_a = dst[6];  g_tVar.S1.seg2_o2_b = dst[7];
    g_tVar.S1.p2_nox = dst[8];  g_tVar.S1.p2_o2 = dst[9];  g_tVar.S1.p3_nox = dst[10]; g_tVar.S1.p3_o2 = dst[11];
    g_tVar.S2.seg1_nox_a = dst[12]; g_tVar.S2.seg1_nox_b = dst[13]; g_tVar.S2.seg1_o2_a = dst[14]; g_tVar.S2.seg1_o2_b = dst[15];
    g_tVar.S2.seg2_nox_a = dst[16]; g_tVar.S2.seg2_nox_b = dst[17]; g_tVar.S2.seg2_o2_a = dst[18]; g_tVar.S2.seg2_o2_b = dst[19];
    g_tVar.S2.p2_nox = dst[20]; g_tVar.S2.p2_o2 = dst[21]; g_tVar.S2.p3_nox = dst[22]; g_tVar.S2.p3_o2 = dst[23];
}
