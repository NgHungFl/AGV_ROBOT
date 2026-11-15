
#ifndef COLOR_SENSOR_TEST_H
#define COLOR_SENSOR_TEST_H

#include "main.h"

HAL_StatusTypeDef ColorSensorTest_Init(I2C_HandleTypeDef *hi2c);

void ColorSensorTest_Run(void);

#endif
