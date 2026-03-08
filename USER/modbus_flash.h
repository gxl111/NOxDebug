/*
 * modbus_flash.h - Internal Flash save/load for Modbus register parameters (VAR_T).
 * Saves P03-P21 (ch0) and P41-P52 (ch1) = 24 floats to FLASH_USER area.
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

#endif
