/*
 * modbus_flash.c - Save/load sensor calibration (24 floats) + blowback interval/duration (S1/S2).
 * Blow stagger 5 min remains in blowback.c (BLOW_STAGGER_SEC), not stored in Flash.
 */
#include "modbus_flash.h"
#include "modbus_slave.h"
#include "app_config.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_flash_ex.h"

#define NUM_FLASH_FLOATS  24
/* After 24 floats: magic + S1_int, S1_dur, S2_int, S2_dur (seconds, uint32 each) */
#define BLOW_FLASH_MAGIC    0x424C5746u   /* 'BLWF' */
#define NUM_FLASH_WORDS     (NUM_FLASH_FLOATS + 1 + 4)

/* Working buffer: 24 floats packed to Flash words before program */
static float s_flash_src[NUM_FLASH_FLOATS];
static uint32_t s_flash_data32[NUM_FLASH_WORDS];

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

static void clamp_duration_u16(uint32_t *p, uint32_t interval)
{
    if (*p < BLOW_DURATION_MIN_S)
        *p = BLOW_DURATION_MIN_S;
    if (interval > 0u && interval <= 65534u && *p >= interval)
        *p = interval - 1u;
}

int InternalFlash_Write(void)
{
    int i;
    uint32_t Address;

    /* 24 floats: S1/S2 seg + p2/p3 */
    s_flash_src[0] = g_tVar.S1.seg1_nox_a; s_flash_src[1] = g_tVar.S1.seg1_nox_b;
    s_flash_src[2] = g_tVar.S1.seg1_o2_a;  s_flash_src[3] = g_tVar.S1.seg1_o2_b;
    s_flash_src[4] = g_tVar.S1.seg2_nox_a; s_flash_src[5] = g_tVar.S1.seg2_nox_b;
    s_flash_src[6] = g_tVar.S1.seg2_o2_a;  s_flash_src[7] = g_tVar.S1.seg2_o2_b;
    s_flash_src[8] = g_tVar.S1.p2_nox; s_flash_src[9] = g_tVar.S1.p2_o2;
    s_flash_src[10] = g_tVar.S1.p3_nox; s_flash_src[11] = g_tVar.S1.p3_o2;
    s_flash_src[12] = g_tVar.S2.seg1_nox_a; s_flash_src[13] = g_tVar.S2.seg1_nox_b;
    s_flash_src[14] = g_tVar.S2.seg1_o2_a;  s_flash_src[15] = g_tVar.S2.seg1_o2_b;
    s_flash_src[16] = g_tVar.S2.seg2_nox_a; s_flash_src[17] = g_tVar.S2.seg2_nox_b;
    s_flash_src[18] = g_tVar.S2.seg2_o2_a;  s_flash_src[19] = g_tVar.S2.seg2_o2_b;
    s_flash_src[20] = g_tVar.S2.p2_nox; s_flash_src[21] = g_tVar.S2.p2_o2;
    s_flash_src[22] = g_tVar.S2.p3_nox; s_flash_src[23] = g_tVar.S2.p3_o2;
    float_to_buf(s_flash_src, s_flash_data32, NUM_FLASH_FLOATS);

    /* Blow: persist as uint32 seconds; 0/0xFFFF kept as-is for stop; duration clamped if interval set */
    {
        uint32_t i0 = (uint32_t)g_tVar.S1.blow_interval;
        uint32_t d0 = (uint32_t)g_tVar.S1.blow_duration;
        uint32_t i1 = (uint32_t)g_tVar.S2.blow_interval;
        uint32_t d1 = (uint32_t)g_tVar.S2.blow_duration;
        if (i0 > 0u && i0 <= 65534u) clamp_duration_u16(&d0, i0);
        if (i1 > 0u && i1 <= 65534u) clamp_duration_u16(&d1, i1);
        s_flash_data32[NUM_FLASH_FLOATS] = BLOW_FLASH_MAGIC;
        s_flash_data32[NUM_FLASH_FLOATS + 1] = i0;
        s_flash_data32[NUM_FLASH_FLOATS + 2] = d0;
        s_flash_data32[NUM_FLASH_FLOATS + 3] = i1;
        s_flash_data32[NUM_FLASH_FLOATS + 4] = d1;
    }

    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t SECTORError;

    HAL_FLASH_Unlock();

    {
        uint32_t NbrOfPage = (FLASH_USER_END_ADDR - FLASH_USER_START_ADDR) / FLASH_PAGE_SIZE;
        EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
        EraseInitStruct.NbPages     = NbrOfPage;
        EraseInitStruct.PageAddress = FLASH_USER_START_ADDR;
        if (HAL_FLASHEx_Erase(&EraseInitStruct, &SECTORError) != HAL_OK) {
            HAL_FLASH_Lock();
            return -1;
        }
    }

    Address = FLASH_USER_START_ADDR;
    for (i = 0; i < NUM_FLASH_WORDS && Address < FLASH_USER_END_ADDR; i++) {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, Address, s_flash_data32[i]) != HAL_OK) {
            HAL_FLASH_Lock();
            return -1;
        }
        Address += 4;
    }

    HAL_FLASH_Lock();

    Address = FLASH_USER_START_ADDR;
    for (i = 0; i < NUM_FLASH_WORDS && Address < FLASH_USER_END_ADDR; i++, Address += 4) {
        if (*(__IO uint32_t *)Address != s_flash_data32[i])
            return 0;
    }
    return 1;
}

int FactoryFlash_ProgramDefaults(void)
{
    /* Ensure blow registers in RAM before write (Register_Init may leave 0 before Blowback_Init) */
    LOCK_VAR();
    g_tVar.S1.blow_interval = (uint16_t)DEFAULT_BLOW_INTERVAL;
    g_tVar.S1.blow_duration = (uint16_t)DEFAULT_BLOW_DURATION;
    g_tVar.S2.blow_interval = (uint16_t)DEFAULT_BLOW_INTERVAL;
    g_tVar.S2.blow_duration = (uint16_t)DEFAULT_BLOW_DURATION;
    UNLOCK_VAR();
    return InternalFlash_Write();
}

void LoadRegistersFromFlash(void)
{
    uint32_t Address = FLASH_USER_START_ADDR;
    uint32_t raw[NUM_FLASH_FLOATS];
    int i;

    for (i = 0; i < NUM_FLASH_FLOATS && Address < FLASH_USER_END_ADDR; i++, Address += 4)
        raw[i] = *(__IO uint32_t *)Address;

    {
        float dst[NUM_FLASH_FLOATS];
        buf_to_float(raw, dst, NUM_FLASH_FLOATS);
        g_tVar.S1.seg1_nox_a = dst[0];  g_tVar.S1.seg1_nox_b = dst[1];  g_tVar.S1.seg1_o2_a = dst[2];  g_tVar.S1.seg1_o2_b = dst[3];
        g_tVar.S1.seg2_nox_a = dst[4];  g_tVar.S1.seg2_nox_b = dst[5];  g_tVar.S1.seg2_o2_a = dst[6];  g_tVar.S1.seg2_o2_b = dst[7];
        g_tVar.S1.p2_nox = dst[8];  g_tVar.S1.p2_o2 = dst[9];  g_tVar.S1.p3_nox = dst[10]; g_tVar.S1.p3_o2 = dst[11];
        g_tVar.S2.seg1_nox_a = dst[12]; g_tVar.S2.seg1_nox_b = dst[13]; g_tVar.S2.seg1_o2_a = dst[14]; g_tVar.S2.seg1_o2_b = dst[15];
        g_tVar.S2.seg2_nox_a = dst[16]; g_tVar.S2.seg2_nox_b = dst[17]; g_tVar.S2.seg2_o2_a = dst[18]; g_tVar.S2.seg2_o2_b = dst[19];
        g_tVar.S2.p2_nox = dst[20]; g_tVar.S2.p2_o2 = dst[21]; g_tVar.S2.p3_nox = dst[22]; g_tVar.S2.p3_o2 = dst[23];
    }

    /* Optional blow block at word 24 */
    if (Address + 20 <= FLASH_USER_END_ADDR) {
        uint32_t magic = *(__IO uint32_t *)Address;
        if (magic == BLOW_FLASH_MAGIC) {
            uint32_t i0 = *(__IO uint32_t *)(Address + 4);
            uint32_t d0 = *(__IO uint32_t *)(Address + 8);
            uint32_t i1 = *(__IO uint32_t *)(Address + 12);
            uint32_t d1 = *(__IO uint32_t *)(Address + 16);
            if (i0 > 65535u) i0 = DEFAULT_BLOW_INTERVAL;
            if (i1 > 65535u) i1 = DEFAULT_BLOW_INTERVAL;
            if (d0 > 65535u) d0 = DEFAULT_BLOW_DURATION;
            if (d1 > 65535u) d1 = DEFAULT_BLOW_DURATION;
            if (i0 > 0u && i0 <= 65534u) clamp_duration_u16(&d0, i0);
            if (i1 > 0u && i1 <= 65534u) clamp_duration_u16(&d1, i1);
            LOCK_VAR();
            g_tVar.S1.blow_interval = (uint16_t)i0;
            g_tVar.S1.blow_duration = (uint16_t)d0;
            g_tVar.S2.blow_interval = (uint16_t)i1;
            g_tVar.S2.blow_duration = (uint16_t)d1;
            UNLOCK_VAR();
        }
    }
}
