
#include "color_sensor_test.h"

#include "tcs34725.h"
#include "bsp_uart.h"

#define COLOR_SENSOR_TEST_INTERVAL_MS    100

static TCS34725_ColorData_t color_data;

static uint32_t last_read_tick = 0;
static uint8_t sensor_ready = 0;


HAL_StatusTypeDef ColorSensorTest_Init(I2C_HandleTypeDef *hi2c)
{
    sensor_ready = 0;

    BSP_UART_Printf("COLOR SENSOR TEST START\r\n");

    if (TCS34725_Init(hi2c) != HAL_OK)
    {
        BSP_UART_Printf("ERROR: TCS34725 INIT FAILED\r\n");
        return HAL_ERROR;
    }

    sensor_ready = 1;
    last_read_tick = HAL_GetTick();

    BSP_UART_Printf("TCS34725 READY\r\n");
    BSP_UART_Printf("R,G,B,C\r\n");

    return HAL_OK;
}


void ColorSensorTest_Run(void)
{
    TCS34725_ColorData_t color;

    if (TCS34725_ReadColor(&color) == HAL_OK)
    {
        const char *color_name = TCS34725_ClassifyColor(color);

        BSP_UART_Printf(
            "%u,%u,%u,%u,%s\r\n",
            color.red,
            color.green,
            color.blue,
            color.clear,
            color_name
        );
    }
    else
    {
        BSP_UART_Printf("COLOR READ ERROR\r\n");
    }

    HAL_Delay(100);
}

