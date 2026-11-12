
#include "line_sensor.h"

// ADC1 duoc khai bao trong main.c
extern ADC_HandleTypeDef hadc1;

// Bo dem DMA cho 5 kenh ADC
static volatile uint16_t adc_buffer[LINE_SENSOR_COUNT] = {0};

// Khoi dong ADC DMA
HAL_StatusTypeDef LineSensor_Init(void)
{
    return HAL_ADC_Start_DMA(
        &hadc1,
        (uint32_t *)(uintptr_t)adc_buffer,
        LINE_SENSOR_COUNT
    );
}

// Doc ca 5 mat do
void LineSensor_Read(uint16_t *data)
{
    if (data == NULL)
        return;

    for (uint8_t i = 0; i < LINE_SENSOR_COUNT; i++)
    {
        data[i] = adc_buffer[i];
    }
}

// Doc mot mat do
uint16_t LineSensor_Get(uint8_t index)
{
    if (index >= LINE_SENSOR_COUNT)
        return 0;

    return adc_buffer[index];
}
