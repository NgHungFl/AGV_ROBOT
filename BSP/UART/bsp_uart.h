
#ifndef BSP_UART_H
#define BSP_UART_H

#include "main.h"
#include <stdint.h>

void BSP_UART_Init(UART_HandleTypeDef *huart);

HAL_StatusTypeDef BSP_UART_SendData(
    const uint8_t *data,
    uint16_t length,
    uint32_t timeout
);

HAL_StatusTypeDef BSP_UART_SendString(
    const char *string,
    uint32_t timeout
);

HAL_StatusTypeDef BSP_UART_Printf(
    const char *format,
    ...
);

HAL_StatusTypeDef BSP_UART_Receive(
    uint8_t *data,
    uint16_t length,
    uint32_t timeout
);

#endif
