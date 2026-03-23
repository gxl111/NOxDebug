/*
 * modbus_flash.h - Internal Flash save/load for Modbus register parameters (VAR_T).
 * Saves 36 floats (S1/S2/S3 calibration) + blow interval/duration for each channel to FLASH_USER area.
 * Loader still accepts legacy image: 24 floats + magic BLWF + S1/S2 blow only.
 * Stagger between channels is fixed in blowback.c (BLOW_STAGGER_SEC), not in Flash.
 */
#ifndef __MODBUS_FLASH_H
#define __MODBUS_FLASH_H
#include <stdint.h>

/* Internal Flash sector for parameter backup (STM32F103 2KB page) */
#define FLASH_USER_START_ADDR   0x08010000
#define FLASH_USER_END_ADDR     0x08010800

/** Write g_tVar params (P03-P21, P41-P52) to Flash. Returns 1 on success, -1 on error. */
int InternalFlash_Write(void);

/** Load g_tVar params from Flash into RAM. */
void LoadRegistersFromFlash(void);

/**
 * Factory program: persist current g_tVar S1/S2/S3 calibration floats and default blow params to Flash.
 * Call after Register_Init() so RAM holds defaults; does not erase other sectors.
 * Returns same as InternalFlash_Write (1 ok, 0 verify fail, -1 error).
 */
int FactoryFlash_ProgramDefaults(void);

#endif
