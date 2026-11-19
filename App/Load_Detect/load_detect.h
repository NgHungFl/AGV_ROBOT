
#ifndef LOAD_DETECT_H
#define LOAD_DETECT_H

#include "main.h"
#include "tcs34725.h"
#include <stdint.h>

typedef enum
{
    LOAD_COLOR_NONE = 0,
    LOAD_COLOR_RED,
    LOAD_COLOR_BLUE
} LoadColor_t;

typedef enum
{
    LOAD_STATUS_EMPTY = 0,      /* chua co hang */
    LOAD_STATUS_CHECKING,       /* co hang, dang xac nhan / doc mau */
    LOAD_STATUS_READY           /* da nhan hang va biet mau */
} LoadStatus_t;

HAL_StatusTypeDef LoadDetect_Init(I2C_HandleTypeDef *hi2c);

/* Bat dau chu ky cho hang moi (giu nguyen baseline) */
void LoadDetect_Reset(void);

/* Goi luc CHUA co hang (xe dang chay): cap nhat muc sang nen */
void LoadDetect_UpdateBaseline(void);

/* Goi khi dang cho hang. Tu gioi han chu ky doc */
LoadStatus_t LoadDetect_Update(void);

LoadColor_t LoadDetect_GetColor(void);

const TCS34725_ColorData_t *LoadDetect_GetSample(void);
uint16_t LoadDetect_GetBaseline(void);

/* Nguong thay doi cua Clear de coi la co hang */
uint16_t LoadDetect_GetThreshold(void);

uint8_t LoadDetect_IsPresent(void);

#endif /* LOAD_DETECT_H */
