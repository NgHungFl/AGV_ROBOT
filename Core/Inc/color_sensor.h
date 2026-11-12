#ifndef COLOR_SENSOR_H
#define COLOR_SENSOR_H

#include "main.h"
#include <stdint.h>

#define TCS34725_ADDRESS       (0x29 << 1)

typedef struct
{
    uint16_t red;
    uint16_t green;
    uint16_t blue;
    uint16_t clear;
} ColorData;

HAL_StatusTypeDef ColorSensor_Init(void);

HAL_StatusTypeDef ColorSensor_Read(ColorData *data);

#endif