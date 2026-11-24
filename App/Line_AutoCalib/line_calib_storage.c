#include "line_calib_storage.h"
#include <string.h>
#include <stddef.h>
#include "stm32f1xx_hal_flash_ex.h"

#define CALIB_MAGIC        0x41474356UL /* 'AGCV' */
#define CALIB_VERSION      3UL /* Two-surface calibration; invalidate old QTR sweep data. */

/* Cấu trúc cố định 32 bytes, chia hết cho 2 để ghi half-word. */
typedef struct
{
    uint32_t magic;
    uint32_t version;
    uint16_t white[LINE_SENSOR_COUNT];
    uint16_t black[LINE_SENSOR_COUNT];
    uint32_t checksum;
} CalibRecord_t;

typedef char calib_record_size_is_32[(sizeof(CalibRecord_t) == 32U) ? 1 : -1];

/* FNV-1a tren 28 byte dau tien, khong phu thuoc CRC peripheral. */
static uint32_t Calib_Checksum(const CalibRecord_t *record)
{
    const uint8_t *p = (const uint8_t *)record;
    uint32_t hash = 2166136261UL;

    for (uint32_t i = 0; i < (uint32_t)offsetof(CalibRecord_t, checksum); i++)
    {
        hash ^= p[i];
        hash *= 16777619UL;
    }
    return hash;
}

static uint8_t Calib_ValuesValid(const uint16_t *white, const uint16_t *black)
{
    for (uint8_t i = 0U; i < LINE_SENSOR_COUNT; i++)
    {
        int32_t span = (int32_t)black[i] - (int32_t)white[i];
        if (span < 0) { span = -span; }
        if ((white[i] == 0U) || (black[i] == 0U) ||
            (white[i] >= 4095U) || (black[i] >= 4095U) ||
            (span < (int32_t)LINE_CALIB_MIN_SPAN))
        {
            return 0U;
        }
    }
    return 1U;
}

uint8_t LineCalibStorage_Load(uint16_t white[LINE_SENSOR_COUNT],
                              uint16_t black[LINE_SENSOR_COUNT])
{
    CalibRecord_t record;
    if ((white == NULL) || (black == NULL)) { return 0U; }

    memcpy(&record, (const void *)LINE_CALIB_FLASH_ADDR, sizeof(record));

    if ((record.magic != CALIB_MAGIC) ||
        (record.version != CALIB_VERSION) ||
        (record.checksum != Calib_Checksum(&record)) ||
        !Calib_ValuesValid(record.white, record.black))
    {
        return 0U;
    }

    memcpy(white, record.white, sizeof(record.white));
    memcpy(black, record.black, sizeof(record.black));
    return 1U;
}

HAL_StatusTypeDef LineCalibStorage_Save(
    const uint16_t white[LINE_SENSOR_COUNT],
    const uint16_t black[LINE_SENSOR_COUNT])
{
    CalibRecord_t record;
    FLASH_EraseInitTypeDef erase = {0};
    uint32_t error_page = 0U;
    HAL_StatusTypeDef result = HAL_OK;
    const uint16_t *hw;
    uint32_t count;

    if ((white == NULL) || (black == NULL) ||
        !Calib_ValuesValid(white, black))
    {
        return HAL_ERROR;
    }

    memset(&record, 0, sizeof(record));
    record.magic   = CALIB_MAGIC;
    record.version = CALIB_VERSION;
    memcpy(record.white, white, sizeof(record.white));
    memcpy(record.black, black, sizeof(record.black));
    record.checksum = Calib_Checksum(&record);

    if (HAL_FLASH_Unlock() != HAL_OK) { return HAL_ERROR; }

    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_WRPERR | FLASH_FLAG_PGERR);

    erase.TypeErase   = FLASH_TYPEERASE_PAGES;
    erase.PageAddress = LINE_CALIB_FLASH_ADDR;
    erase.NbPages     = 1U;
    erase.Banks       = FLASH_BANK_1;

    result = HAL_FLASHEx_Erase(&erase, &error_page);

    /* Ghi magic sau cung: khi mat dien giua chung, record se khong hop le. */
    hw = (const uint16_t *)&record;
    count = (uint32_t)(sizeof(record) / sizeof(uint16_t));
    if (result == HAL_OK)
    {
        for (uint32_t i = 2U; i < count; i++)
        {
            if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,
                                  LINE_CALIB_FLASH_ADDR + i * 2U,
                                  hw[i]) != HAL_OK)
            {
                result = HAL_ERROR;
                break;
            }
        }
    }
    if (result == HAL_OK)
    {
        for (uint32_t i = 0U; i < 2U; i++)
        {
            if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,
                                  LINE_CALIB_FLASH_ADDR + i * 2U,
                                  hw[i]) != HAL_OK)
            {
                result = HAL_ERROR;
                break;
            }
        }
    }

    (void)HAL_FLASH_Lock();

    /* Xac minh lai record vua ghi, de khong tu nhan la da luu thanh cong. */
    if (result == HAL_OK)
    {
        uint16_t check_white[LINE_SENSOR_COUNT], check_black[LINE_SENSOR_COUNT];
        if (!LineCalibStorage_Load(check_white, check_black) ||
            (memcmp(check_white, white, sizeof(check_white)) != 0) ||
            (memcmp(check_black, black, sizeof(check_black)) != 0))
        {
            result = HAL_ERROR;
        }
    }
    return result;
}
