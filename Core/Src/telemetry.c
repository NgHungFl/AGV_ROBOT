
#include "telemetry.h"
#include <stdio.h>

// USART3 duoc khai bao trong main.c
extern UART_HandleTypeDef huart3;

void Telemetry_SendLine(const uint16_t *data)
{
    if (data == NULL)
        return;

    char buffer[80];

    int length = snprintf(
        buffer,
        sizeof(buffer),
        "%u,%u,%u,%u,%u\r\n",
        (unsigned int)data[0],
        (unsigned int)data[1],
        (unsigned int)data[2],
        (unsigned int)data[3],
        (unsigned int)data[4]
    );

    if (length > 0 && length < sizeof(buffer))
    {
        HAL_UART_Transmit(
            &huart3,
            (uint8_t *)buffer,
            (uint16_t)length,
            100
        );
    }
}

#include <stdio.h>
#include <string.h>
#include "telemetry.h"

extern UART_HandleTypeDef huart3;

void Telemetry_SendColor(
    uint16_t red,
    uint16_t green,
    uint16_t blue,
    uint16_t clear,
    const char *color_name
)
{
    char buffer[100];

    snprintf(buffer, sizeof(buffer),
             "COLOR,%u,%u,%u,%u,%s\r\n",
             red, green, blue, clear, color_name);

    HAL_UART_Transmit(
        &huart3,
        (uint8_t *)buffer,
        strlen(buffer),
        100
    );
}