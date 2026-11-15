
#ifndef TCS34725_H
#define TCS34725_H

#include "main.h"
#include <stdint.h>

typedef struct
{
    uint16_t clear;
    uint16_t red;
    uint16_t green;
    uint16_t blue;
} TCS34725_ColorData_t;

HAL_StatusTypeDef TCS34725_Init(I2C_HandleTypeDef *hi2c);

HAL_StatusTypeDef TCS34725_ReadColor(
    TCS34725_ColorData_t *color
);

const char* TCS34725_ClassifyColor(TCS34725_ColorData_t color);

#endif
