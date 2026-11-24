
#include "line_sensor_test.h"

#include "line_sensor.h"
#include "bsp_uart.h"

#include "main.h"

/* Chu ky hien thi */
#define LINE_TEST_INTERVAL_MS 100U

static uint32_t last_print_time = 0;
static uint32_t last_sample_time = 0;

/* Khoi tao */
void LineSensorTest_Init(void)
{
    last_print_time = HAL_GetTick();
    last_sample_time = last_print_time;

    BSP_UART_Printf("\r\n");
    BSP_UART_Printf("========================================\r\n");
    BSP_UART_Printf("        LINE SENSOR 5 CHANNEL TEST     \r\n");
    BSP_UART_Printf("========================================\r\n");

    BSP_UART_Printf("Order: PA4 PA3 PA2 PA1 PA0\r\n");

    BSP_UART_Printf("----------------------------------------\r\n");
}

/* In bieu dien 5 mat */
static void LineSensorTest_PrintBar(
    const LineSensor_Data_t *data
)
{
    BSP_UART_Printf("LINE : ");

    for (uint8_t i = 0; i < LINE_SENSOR_COUNT; i++)
    {
        if (data->value[i] >= LINE_DETECT_THRESHOLD)
        {
            BSP_UART_Printf("[X] ");
        }
        else
        {
            BSP_UART_Printf("[ ] ");
        }
    }

    BSP_UART_Printf("\r\n");
}

/* Chuong trinh test */
void LineSensorTest_Run(void)
{
    const LineSensor_Data_t *data;
    uint32_t now = HAL_GetTick();

    /* Lay mau theo chu ky dieu khien truoc khi loc vi tri. */
    if ((now - last_sample_time) < ROBOT_CONTROL_PERIOD_MS)
    {
        return;
    }
    last_sample_time = now;
    LineSensor_Update();

    /* Gioi han toc do UART */
    if ((now - last_print_time) < LINE_TEST_INTERVAL_MS)
    {
        return;
    }

    last_print_time = now;

    data = LineSensor_GetData();

    /* RAW ADC */
    BSP_UART_Printf(
        "RAW : %4u | %4u | %4u | %4u | %4u\r\n",
        data->raw[0],
        data->raw[1],
        data->raw[2],
        data->raw[3],
        data->raw[4]
    );

    /* VALUE NORMALIZED */
    BSP_UART_Printf(
        "VAL : %4u | %4u | %4u | %4u | %4u\r\n",
        data->value[0],
        data->value[1],
        data->value[2],
        data->value[3],
        data->value[4]
    );

    /* Vi tri line */
    if (data->status == LINE_DETECTED)
    {
        BSP_UART_Printf("POS_RAW : %ld\r\n", (long)data->position);
        BSP_UART_Printf(
            "POS : %ld\r\n",
            (long)data->position_filtered
        );

        BSP_UART_Printf("STATE: LINE DETECTED\r\n");
    }
    else
    {
        BSP_UART_Printf("POS_RAW : ---\r\n");
        BSP_UART_Printf("POS : ---\r\n");
        BSP_UART_Printf("STATE: LINE LOST\r\n");
    }

    /* Bieu dien truc quan */
    LineSensorTest_PrintBar(data);

    BSP_UART_Printf("----------------------------------------\r\n");
}
