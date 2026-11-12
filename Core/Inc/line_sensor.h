
#ifndef LINE_SENSOR_H
#define LINE_SENSOR_H

#include "main.h"
#include <stdint.h>

#define LINE_SENSOR_COUNT  5

// Khoi dong ADC va DMA
HAL_StatusTypeDef LineSensor_Init(void);

// Lay du lieu cua ca 5 mat do
void LineSensor_Read(uint16_t *data);

// Lay du lieu cua mot mat do
uint16_t LineSensor_Get(uint8_t index);

#endif
