/*
 * modbus_flash.c - Save/load sensor calibration (S1/S2/S3) + blowback interval/duration per channel.
 * Layout v2: 36 floats + magic BLW3 + 6 uint32 (S1/S2/S3 blow int/dur).
 * Older format: 24 floats + magic BLWF + 4 uint32 (S1/S2 blow only) — still loaded if word[36] != V2 magic.
 * Blow stagger remains in blowback.c (BLOW_STAGGER_SEC), not in Flash.
 */
#include "modbus_flash.h"
#include "modbus_slave.h"
#include "app_config.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_flash_ex.h"

#define NUM_FLASH_FLOATS_V1   24
#define NUM_FLASH_FLOATS      36

/* v1: magic at word index 24 after 24 floats */
#define BLOW_FLASH_MAGIC      0x424C5746u   /* 'BLWF' */
/* v2: magic at word index 36 after 36 floats */
#define BLOW_FLASH_MAGIC_V2   0x424C5733u   /* 'BL3' 3-channel layout */

#define BLOW_WORD_IDX_V1_MAGIC   24u
#define BLOW_WORD_IDX_V2_MAGIC   36u
#define NUM_FLASH_WORDS_V1       (NUM_FLASH_FLOATS_V1 + 1u + 4u)
#define NUM_FLASH_WORDS_V2       (NUM_FLASH_FLOATS + 1u + 6u)

#define FLASH_U32_PTR(idx)  ((__IO uint32_t *)(FLASH_USER_START_ADDR + (uint32_t)(idx) * 4u))

static float s_flash_src[NUM_FLASH_FLOATS];
static uint32_t s_flash_data32[NUM_FLASH_WORDS_V2];

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
    if (interval > BLOW_DURATION_MIN_S && interval <= 65534u && *p >= interval)
        *p = interval - 1u;
}

static void pack_cal_floats_to_buf(uint32_t *dst)
{
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
    s_flash_src[24] = g_tVar.S3.seg1_nox_a; s_flash_src[25] = g_tVar.S3.seg1_nox_b;
    s_flash_src[26] = g_tVar.S3.seg1_o2_a;  s_flash_src[27] = g_tVar.S3.seg1_o2_b;
    s_flash_src[28] = g_tVar.S3.seg2_nox_a; s_flash_src[29] = g_tVar.S3.seg2_nox_b;
    s_flash_src[30] = g_tVar.S3.seg2_o2_a;  s_flash_src[31] = g_tVar.S3.seg2_o2_b;
    s_flash_src[32] = g_tVar.S3.p2_nox; s_flash_src[33] = g_tVar.S3.p2_o2;
    s_flash_src[34] = g_tVar.S3.p3_nox; s_flash_src[35] = g_tVar.S3.p3_o2;
    float_to_buf(s_flash_src, dst, NUM_FLASH_FLOATS);
}

int InternalFlash_Write(void)
{
    int i;
    uint32_t Address;

    pack_cal_floats_to_buf(s_flash_data32);

    {
        uint32_t i0 = (uint32_t)g_tVar.S1.blow_interval;
        uint32_t d0 = (uint32_t)g_tVar.S1.blow_duration;
        uint32_t i1 = (uint32_t)g_tVar.S2.blow_interval;
        uint32_t d1 = (uint32_t)g_tVar.S2.blow_duration;
        uint32_t i2 = (uint32_t)g_tVar.S3.blow_interval;
        uint32_t d2 = (uint32_t)g_tVar.S3.blow_duration;
        if (i0 > 0u && i0 <= 65534u) clamp_duration_u16(&d0, i0);
        if (i1 > 0u && i1 <= 65534u) clamp_duration_u16(&d1, i1);
        if (i2 > 0u && i2 <= 65534u) clamp_duration_u16(&d2, i2);
        s_flash_data32[BLOW_WORD_IDX_V2_MAGIC] = BLOW_FLASH_MAGIC_V2;
        s_flash_data32[37] = i0;
        s_flash_data32[38] = d0;
        s_flash_data32[39] = i1;
        s_flash_data32[40] = d1;
        s_flash_data32[41] = i2;
        s_flash_data32[42] = d2;
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
    for (i = 0; i < (int)NUM_FLASH_WORDS_V2 && Address < FLASH_USER_END_ADDR; i++) {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, Address, s_flash_data32[i]) != HAL_OK) {
            HAL_FLASH_Lock();
            return -1;
        }
        Address += 4;
    }

    HAL_FLASH_Lock();

    Address = FLASH_USER_START_ADDR;
    for (i = 0; i < (int)NUM_FLASH_WORDS_V2 && Address < FLASH_USER_END_ADDR; i++, Address += 4) {
        if (*(__IO uint32_t *)Address != s_flash_data32[i])
            return 0;
    }
    return 1;
}

int FactoryFlash_ProgramDefaults(void)
{
    LOCK_VAR();
    g_tVar.S1.blow_interval = (uint16_t)DEFAULT_BLOW_INTERVAL;
    g_tVar.S1.blow_duration = (uint16_t)DEFAULT_BLOW_DURATION;
    g_tVar.S2.blow_interval = (uint16_t)DEFAULT_BLOW_INTERVAL;
    g_tVar.S2.blow_duration = (uint16_t)DEFAULT_BLOW_DURATION;
    g_tVar.S3.blow_interval = (uint16_t)DEFAULT_BLOW_INTERVAL;
    g_tVar.S3.blow_duration = (uint16_t)DEFAULT_BLOW_DURATION;
    UNLOCK_VAR();
    return InternalFlash_Write();
}

void LoadRegistersFromFlash(void)
{
    uint32_t raw[36];
    float dst[36];
    int i;

    if (FLASH_USER_START_ADDR + NUM_FLASH_WORDS_V1 * 4u > FLASH_USER_END_ADDR)
        return;

    uint32_t magic_v2 = *FLASH_U32_PTR(BLOW_WORD_IDX_V2_MAGIC);
    uint32_t magic_v1 = *FLASH_U32_PTR(BLOW_WORD_IDX_V1_MAGIC);

    if (magic_v2 == BLOW_FLASH_MAGIC_V2 &&
        FLASH_USER_START_ADDR + NUM_FLASH_WORDS_V2 * 4u <= FLASH_USER_END_ADDR) {
        for (i = 0; i < NUM_FLASH_FLOATS; i++)
            raw[i] = *FLASH_U32_PTR((uint32_t)i);
        buf_to_float(raw, dst, NUM_FLASH_FLOATS);
        LOCK_VAR();
        g_tVar.S1.seg1_nox_a = dst[0];  g_tVar.S1.seg1_nox_b = dst[1];  g_tVar.S1.seg1_o2_a = dst[2];  g_tVar.S1.seg1_o2_b = dst[3];
        g_tVar.S1.seg2_nox_a = dst[4];  g_tVar.S1.seg2_nox_b = dst[5];  g_tVar.S1.seg2_o2_a = dst[6];  g_tVar.S1.seg2_o2_b = dst[7];
        g_tVar.S1.p2_nox = dst[8];  g_tVar.S1.p2_o2 = dst[9];  g_tVar.S1.p3_nox = dst[10]; g_tVar.S1.p3_o2 = dst[11];
        g_tVar.S2.seg1_nox_a = dst[12]; g_tVar.S2.seg1_nox_b = dst[13]; g_tVar.S2.seg1_o2_a = dst[14]; g_tVar.S2.seg1_o2_b = dst[15];
        g_tVar.S2.seg2_nox_a = dst[16]; g_tVar.S2.seg2_nox_b = dst[17]; g_tVar.S2.seg2_o2_a = dst[18]; g_tVar.S2.seg2_o2_b = dst[19];
        g_tVar.S2.p2_nox = dst[20]; g_tVar.S2.p2_o2 = dst[21]; g_tVar.S2.p3_nox = dst[22]; g_tVar.S2.p3_o2 = dst[23];
        g_tVar.S3.seg1_nox_a = dst[24]; g_tVar.S3.seg1_nox_b = dst[25]; g_tVar.S3.seg1_o2_a = dst[26]; g_tVar.S3.seg1_o2_b = dst[27];
        g_tVar.S3.seg2_nox_a = dst[28]; g_tVar.S3.seg2_nox_b = dst[29]; g_tVar.S3.seg2_o2_a = dst[30]; g_tVar.S3.seg2_o2_b = dst[31];
        g_tVar.S3.p2_nox = dst[32]; g_tVar.S3.p2_o2 = dst[33]; g_tVar.S3.p3_nox = dst[34]; g_tVar.S3.p3_o2 = dst[35];
        {
            uint32_t i0 = *FLASH_U32_PTR(37);
            uint32_t d0 = *FLASH_U32_PTR(38);
            uint32_t i1 = *FLASH_U32_PTR(39);
            uint32_t d1 = *FLASH_U32_PTR(40);
            uint32_t i2 = *FLASH_U32_PTR(41);
            uint32_t d2 = *FLASH_U32_PTR(42);
            if (i0 > 65535u) i0 = DEFAULT_BLOW_INTERVAL;
            if (i1 > 65535u) i1 = DEFAULT_BLOW_INTERVAL;
            if (i2 > 65535u) i2 = DEFAULT_BLOW_INTERVAL;
            if (d0 > 65535u) d0 = DEFAULT_BLOW_DURATION;
            if (d1 > 65535u) d1 = DEFAULT_BLOW_DURATION;
            if (d2 > 65535u) d2 = DEFAULT_BLOW_DURATION;
            if (i0 > 0u && i0 <= 65534u) clamp_duration_u16(&d0, i0);
            if (i1 > 0u && i1 <= 65534u) clamp_duration_u16(&d1, i1);
            if (i2 > 0u && i2 <= 65534u) clamp_duration_u16(&d2, i2);
            g_tVar.S1.blow_interval = (uint16_t)i0;
            g_tVar.S1.blow_duration = (uint16_t)d0;
            g_tVar.S2.blow_interval = (uint16_t)i1;
            g_tVar.S2.blow_duration = (uint16_t)d1;
            g_tVar.S3.blow_interval = (uint16_t)i2;
            g_tVar.S3.blow_duration = (uint16_t)d2;
        }
        UNLOCK_VAR();
        return;
    }

    if (magic_v1 == BLOW_FLASH_MAGIC &&
        FLASH_USER_START_ADDR + NUM_FLASH_WORDS_V1 * 4u <= FLASH_USER_END_ADDR) {
        for (i = 0; i < NUM_FLASH_FLOATS_V1; i++)
            raw[i] = *FLASH_U32_PTR((uint32_t)i);
        buf_to_float(raw, dst, NUM_FLASH_FLOATS_V1);
        LOCK_VAR();
        g_tVar.S1.seg1_nox_a = dst[0];  g_tVar.S1.seg1_nox_b = dst[1];  g_tVar.S1.seg1_o2_a = dst[2];  g_tVar.S1.seg1_o2_b = dst[3];
        g_tVar.S1.seg2_nox_a = dst[4];  g_tVar.S1.seg2_nox_b = dst[5];  g_tVar.S1.seg2_o2_a = dst[6];  g_tVar.S1.seg2_o2_b = dst[7];
        g_tVar.S1.p2_nox = dst[8];  g_tVar.S1.p2_o2 = dst[9];  g_tVar.S1.p3_nox = dst[10]; g_tVar.S1.p3_o2 = dst[11];
        g_tVar.S2.seg1_nox_a = dst[12]; g_tVar.S2.seg1_nox_b = dst[13]; g_tVar.S2.seg1_o2_a = dst[14]; g_tVar.S2.seg1_o2_b = dst[15];
        g_tVar.S2.seg2_nox_a = dst[16]; g_tVar.S2.seg2_nox_b = dst[17]; g_tVar.S2.seg2_o2_a = dst[18]; g_tVar.S2.seg2_o2_b = dst[19];
        g_tVar.S2.p2_nox = dst[20]; g_tVar.S2.p2_o2 = dst[21]; g_tVar.S2.p3_nox = dst[22]; g_tVar.S2.p3_o2 = dst[23];
        {
            uint32_t i0 = *FLASH_U32_PTR(25);
            uint32_t d0 = *FLASH_U32_PTR(26);
            uint32_t i1 = *FLASH_U32_PTR(27);
            uint32_t d1 = *FLASH_U32_PTR(28);
            if (i0 > 65535u) i0 = DEFAULT_BLOW_INTERVAL;
            if (i1 > 65535u) i1 = DEFAULT_BLOW_INTERVAL;
            if (d0 > 65535u) d0 = DEFAULT_BLOW_DURATION;
            if (d1 > 65535u) d1 = DEFAULT_BLOW_DURATION;
            if (i0 > 0u && i0 <= 65534u) clamp_duration_u16(&d0, i0);
            if (i1 > 0u && i1 <= 65534u) clamp_duration_u16(&d1, i1);
            g_tVar.S1.blow_interval = (uint16_t)i0;
            g_tVar.S1.blow_duration = (uint16_t)d0;
            g_tVar.S2.blow_interval = (uint16_t)i1;
            g_tVar.S2.blow_duration = (uint16_t)d1;
        }
        UNLOCK_VAR();
        return;
    }

    /* No recognized magic: leave g_tVar from Register_Init (avoid interpreting blank flash as floats). */
}
