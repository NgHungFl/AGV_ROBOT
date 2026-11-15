
#include "bsp_uart.h"

#include <stdio.h>
#include <string.h>
#include <stdarg.h>

static UART_HandleTypeDef *uart_handle = NULL;

void BSP_UART_Init(UART_HandleTypeDef *huart)
{
    uart_handle = huart;
}


HAL_StatusTypeDef BSP_UART_SendData(
    const uint8_t *data,
    uint16_t length,
    uint32_t timeout
)
{
    if ((uart_handle == NULL) || (data == NULL) || (length == 0))
    {
        return HAL_ERROR;
    }

    return HAL_UART_Transmit(
        uart_handle,
        (uint8_t *)data,
        length,
        timeout
    );
}


HAL_StatusTypeDef BSP_UART_SendString(
    const char *string,
    uint32_t timeout
)
{
    if ((uart_handle == NULL) || (string == NULL))
    {
        return HAL_ERROR;
    }

    return BSP_UART_SendData(
        (const uint8_t *)string,
        (uint16_t)strlen(string),
        timeout
    );
}


HAL_StatusTypeDef BSP_UART_Printf(
    const char *format,
    ...
)
{
    char buffer[256];

    if ((uart_handle == NULL) || (format == NULL))
    {
        return HAL_ERROR;
    }

    va_list args;
    va_start(args, format);

    int length = vsnprintf(buffer, sizeof(buffer), format, args);

    va_end(args);

    if (length < 0)
    {
        return HAL_ERROR;
    }

    if (length >= (int)sizeof(buffer))
    {
        length = sizeof(buffer) - 1;
    }

    if (length == 0)
    {
        return HAL_OK;
    }

    return BSP_UART_SendData(
        (const uint8_t *)buffer,
        (uint16_t)length,
        100
    );
}


HAL_StatusTypeDef BSP_UART_Receive(
    uint8_t *data,
    uint16_t length,
    uint32_t timeout
)
{
    if ((uart_handle == NULL) || (data == NULL) || (length == 0))
    {
        return HAL_ERROR;
    }

    return HAL_UART_Receive(
        uart_handle,
        data,
        length,
        timeout
    );
}

