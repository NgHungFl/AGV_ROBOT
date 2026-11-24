#ifndef LINE_CALIB_STORAGE_H
#define LINE_CALIB_STORAGE_H

#include "main.h"
#include "line_sensor.h"

/* Trang cuoi cua FLASH 64 KB: 0x0800FC00-0x0800FFFF.
 * Phai reserve bang cach doi FLASH LENGTH = 63K trong linker script.
 */
#define LINE_CALIB_FLASH_ADDR  0x0800FC00UL
#define LINE_CALIB_MIN_SPAN    LINE_AUTO_CALIB_MIN_SPAN

/* 1: du lieu da luu hop le, 0: khong co hoac bi hong. */
uint8_t LineCalibStorage_Load(uint16_t white[LINE_SENSOR_COUNT],
                              uint16_t black[LINE_SENSOR_COUNT]);

/* Ghi 1 lan SAU KHI da calib xong. Khong duoc goi trong ngat TIM2. */
HAL_StatusTypeDef LineCalibStorage_Save(
    const uint16_t white[LINE_SENSOR_COUNT],
    const uint16_t black[LINE_SENSOR_COUNT]);

#endif
