
#include "tcs34725.h"

/* TCS34725 I2C address: 0x29 */
#define TCS34725_I2C_ADDR       (0x29 << 1)

/* Register addresses */
#define TCS34725_REG_ENABLE     0x00
#define TCS34725_REG_ATIME      0x01
#define TCS34725_REG_CONTROL    0x0F
#define TCS34725_REG_ID         0x12
#define TCS34725_REG_CDATAL     0x14

/* Command bits */
#define TCS34725_CMD_BIT        0x80
#define TCS34725_AUTO_INCREMENT 0x20

/* Enable register */
#define TCS34725_ENABLE_PON     0x01
#define TCS34725_ENABLE_AEN     0x02

/* Sensor configuration */
#define TCS34725_ATIME_VALUE    0xEB  // ~50.4 ms
#define TCS34725_GAIN_VALUE     0x01  // 4x gain

static I2C_HandleTypeDef *tcs34725_i2c = NULL;


/* Write one register */
static HAL_StatusTypeDef TCS34725_WriteRegister(
    uint8_t reg,
    uint8_t value
)
{
    uint8_t command = TCS34725_CMD_BIT | reg;

    return HAL_I2C_Mem_Write(
        tcs34725_i2c,
        TCS34725_I2C_ADDR,
        command,
        I2C_MEMADD_SIZE_8BIT,
        &value,
        1,
        100
    );
}


/* Read one register */
static HAL_StatusTypeDef TCS34725_ReadRegister(
    uint8_t reg,
    uint8_t *value
)
{
    uint8_t command = TCS34725_CMD_BIT | reg;

    return HAL_I2C_Mem_Read(
        tcs34725_i2c,
        TCS34725_I2C_ADDR,
        command,
        I2C_MEMADD_SIZE_8BIT,
        value,
        1,
        100
    );
}


/* Initialize sensor */
HAL_StatusTypeDef TCS34725_Init(I2C_HandleTypeDef *hi2c)
{
    uint8_t sensor_id = 0;

    if (hi2c == NULL)
    {
        return HAL_ERROR;
    }

    tcs34725_i2c = hi2c;

    /* Check whether sensor responds */
    if (HAL_I2C_IsDeviceReady(
            tcs34725_i2c,
            TCS34725_I2C_ADDR,
            3,
            100) != HAL_OK)
    {
        return HAL_ERROR;
    }

    /* Read sensor ID */
    if (TCS34725_ReadRegister(
            TCS34725_REG_ID,
            &sensor_id) != HAL_OK)
    {
        return HAL_ERROR;
    }

    /* Common TCS3472x compatible IDs */
    if ((sensor_id != 0x44) && (sensor_id != 0x4D))
    {
        return HAL_ERROR;
    }

    /* Set integration time */
    if (TCS34725_WriteRegister(
            TCS34725_REG_ATIME,
            TCS34725_ATIME_VALUE) != HAL_OK)
    {
        return HAL_ERROR;
    }

    /* Set gain */
    if (TCS34725_WriteRegister(
            TCS34725_REG_CONTROL,
            TCS34725_GAIN_VALUE) != HAL_OK)
    {
        return HAL_ERROR;
    }

    /* Power ON */
    if (TCS34725_WriteRegister(
            TCS34725_REG_ENABLE,
            TCS34725_ENABLE_PON) != HAL_OK)
    {
        return HAL_ERROR;
    }

    HAL_Delay(3);

    /* Enable ADC and RGBC measurement */
    if (TCS34725_WriteRegister(
            TCS34725_REG_ENABLE,
            TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN) != HAL_OK)
    {
        return HAL_ERROR;
    }

    /* Wait for first measurement */
    HAL_Delay(60);

    return HAL_OK;
}


/* Read Clear, Red, Green, Blue */
HAL_StatusTypeDef TCS34725_ReadColor(
    TCS34725_ColorData_t *color
)
{
    uint8_t command = TCS34725_CMD_BIT |
                      TCS34725_AUTO_INCREMENT |
                      TCS34725_REG_CDATAL;

    uint8_t data[8];

    if ((tcs34725_i2c == NULL) || (color == NULL))
    {
        return HAL_ERROR;
    }

    if (HAL_I2C_Mem_Read(
            tcs34725_i2c,
            TCS34725_I2C_ADDR,
            command,
            I2C_MEMADD_SIZE_8BIT,
            data,
            sizeof(data),
            100) != HAL_OK)
    {
        return HAL_ERROR;
    }

    /* TCS34725 data order: C, R, G, B */
    color->clear = ((uint16_t)data[1] << 8) | data[0];
    color->red   = ((uint16_t)data[3] << 8) | data[2];
    color->green = ((uint16_t)data[5] << 8) | data[4];
    color->blue  = ((uint16_t)data[7] << 8) | data[6];

    return HAL_OK;
}
/* xác định phần trăm để chọn màu*/
const char* TCS34725_ClassifyColor(TCS34725_ColorData_t color)
{
    uint32_t sum = color.red + color.green + color.blue;

    if (sum < 140)
    {
        return "UNKNOWN";
    }

    float red_ratio = (float)color.red / sum;

    if (red_ratio > 0.33f)
    {
        return "RED";
    }
    else
    {
        return "BLUE";
    }
}