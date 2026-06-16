/*
 * modbus_flash.h - Internal Flash save/load for Modbus register parameters (VAR_T).
 * Saves 36 floats (S1/S2/S3 calibration) + blow interval/duration for each channel to FLASH_USER area.
 * Supports loading older image: 24 floats + magic BLWF + S1/S2 blow only.
 * Blowback phase split is fixed in blowback.c/app_config.h (BLOW_PHASE_DIVISOR), not in Flash.
 */
#ifndef __MODBUS_FLASH_H
#define __MODBUS_FLASH_H
#include <stdint.h>

/* Internal Flash sector for parameter backup: last 2KB page of STM32F103RC 256KB Flash. */
#define FLASH_USER_START_ADDR   0x0803F800
#define FLASH_USER_END_ADDR     0x08040000

/** Write S1/S2/S3 calibration floats plus blowback interval/duration to Flash. Returns 1 on success, -1 on error. */
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
