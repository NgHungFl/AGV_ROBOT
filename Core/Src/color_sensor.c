#include "color_sensor.h"

extern I2C_HandleTypeDef hi2c1;

// Các thanh ghi TCS34725
#define TCS_COMMAND_BIT     0x80

#define TCS_ENABLE          0x00
#define TCS_ATIME           0x01
#define TCS_CONTROL         0x0F

#define TCS_CDATAL          0x14
#define TCS_RDATAL          0x16
#define TCS_GDATAL          0x18
#define TCS_BDATAL          0x1A

// Hàm ghi 1 byte vào thanh ghi
static HAL_StatusTypeDef ColorSensor_Write(uint8_t reg, uint8_t value)
{
    uint8_t command = TCS_COMMAND_BIT | reg;

    return HAL_I2C_Mem_Write(
        &hi2c1,
        TCS34725_ADDRESS,
        command,
        I2C_MEMADD_SIZE_8BIT,
        &value,
        1,
        100
    );
}

// Hàm đọc 2 byte dữ liệu
static HAL_StatusTypeDef ColorSensor_Read16(uint8_t reg, uint16_t *value)
{
    uint8_t buffer[2];

    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(
        &hi2c1,
        TCS34725_ADDRESS,
        TCS_COMMAND_BIT | reg,
        I2C_MEMADD_SIZE_8BIT,
        buffer,
        2,
        100
    );

    if (status == HAL_OK)
    {
        *value = (uint16_t)buffer[0] |
                 ((uint16_t)buffer[1] << 8);
    }

    return status;
}

// Khởi tạo cảm biến
HAL_StatusTypeDef ColorSensor_Init(void)
{
    HAL_StatusTypeDef status;

    // Kiểm tra thiết bị
    status = HAL_I2C_IsDeviceReady(
        &hi2c1,
        TCS34725_ADDRESS,
        3,
        100
    );

    if (status != HAL_OK)
        return status;

    // Thời gian tích hợp: 154 ms
    status = ColorSensor_Write(TCS_ATIME, 0xC0);
    if (status != HAL_OK)
        return status;

    // Gain: 4x
    status = ColorSensor_Write(TCS_CONTROL, 0x01);
    if (status != HAL_OK)
        return status;

    // Bật nguồn cảm biến
    status = ColorSensor_Write(TCS_ENABLE, 0x01);
    if (status != HAL_OK)
        return status;

    HAL_Delay(3);

    // Bật ADC đo màu
    status = ColorSensor_Write(TCS_ENABLE, 0x03);

    return status;
}

// Đọc dữ liệu RGB và Clear
HAL_StatusTypeDef ColorSensor_Read(ColorData *data)
{
    HAL_StatusTypeDef status;

    if (data == NULL)
        return HAL_ERROR;

    status = ColorSensor_Read16(TCS_CDATAL, &data->clear);
    if (status != HAL_OK) return status;

    status = ColorSensor_Read16(TCS_RDATAL, &data->red);
    if (status != HAL_OK) return status;

    status = ColorSensor_Read16(TCS_GDATAL, &data->green);
    if (status != HAL_OK) return status;

    status = ColorSensor_Read16(TCS_BDATAL, &data->blue);
    if (status != HAL_OK) return status;

    return HAL_OK;
}