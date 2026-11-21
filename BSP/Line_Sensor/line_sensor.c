
#include "line_sensor.h"
#include <string.h>

/* =================================================
 * ADC DMA BUFFER
 *
 * Rank 1: PA0
 * Rank 2: PA1
 * Rank 3: PA2
 * Rank 4: PA3
 * Rank 5: PA4
 *
 * Thu tu vat ly S1 -> S5:
 * PA4 -> PA3 -> PA2 -> PA1 -> PA0
 * ================================================= */

/* LINE_OVERSAMPLE lan quet lien tiep, moi lan quet 5 kenh:
   adc_buffer[n * 5 + kenh] */
static uint16_t adc_buffer[LINE_SENSOR_COUNT * LINE_OVERSAMPLE];

/* Du lieu cam bien */
static LineSensor_Data_t sensor_data;

static volatile LinePreference_t line_pref = LINE_PREF_NONE;

/* ADC handle */
static ADC_HandleTypeDef *adc_handle = NULL;

/* Calibration BLACK */
static const uint16_t black_calib[LINE_SENSOR_COUNT] =
{
    LINE_S1_BLACK,
    LINE_S2_BLACK,
    LINE_S3_BLACK,
    LINE_S4_BLACK,
    LINE_S5_BLACK
};

/* Calibration WHITE */
static const uint16_t white_calib[LINE_SENSOR_COUNT] =
{
    LINE_S1_WHITE,
    LINE_S2_WHITE,
    LINE_S3_WHITE,
    LINE_S4_WHITE,
    LINE_S5_WHITE
};

/* Trong so vi tri */
static const int16_t weights[LINE_SENSOR_COUNT] =
{
    LINE_WEIGHT_S1,
    LINE_WEIGHT_S2,
    LINE_WEIGHT_S3,
    LINE_WEIGHT_S4,
    LINE_WEIGHT_S5
};

/* =================================================
 * CHUAN HOA ADC
 *
 * BLACK = 1000
 * WHITE = 0
 *
 * Ho tro ca truong hop:
 * ADC den > ADC trang
 * ADC den < ADC trang
 * ================================================= */

static uint16_t LineSensor_Normalize(
    uint16_t raw,
    uint16_t black,
    uint16_t white
)
{
    int32_t numerator;
    int32_t denominator;
    int32_t result;

    if (black == white)
    {
        return 0;
    }

    numerator =
        ((int32_t)raw - (int32_t)white) * 1000;

    denominator =
        (int32_t)black - (int32_t)white;

    result = numerator / denominator;

    if (result < 0)
    {
        result = 0;
    }

    if (result > 1000)
    {
        result = 1000;
    }

    return (uint16_t)result;
}

/* Gia tri dong gop vao vi tri (trong so mem) */
static uint32_t LineSensor_Contribution(uint16_t value)
{
#if (LINE_SOFT_WEIGHT != 0)
    return (value > LINE_SOFT_FLOOR) ? (uint32_t)(value - LINE_SOFT_FLOOR) : 0U;
#else
    return (value >= LINE_DETECT_THRESHOLD) ? (uint32_t)value : 0U;
#endif
}

/* =================================================
 * KHOI TAO LINE SENSOR
 * ================================================= */

HAL_StatusTypeDef LineSensor_Init(ADC_HandleTypeDef *hadc)
{
    if (hadc == NULL)
    {
        return HAL_ERROR;
    }

    adc_handle = hadc;

    memset(adc_buffer, 0, sizeof(adc_buffer));
    memset(&sensor_data, 0, sizeof(sensor_data));

    /* Hieu chuan ADC cho STM32F1 */
    if (HAL_ADCEx_Calibration_Start(adc_handle) != HAL_OK)
    {
        adc_handle = NULL;
        return HAL_ERROR;
    }

    /* Bat ADC DMA */
    if (HAL_ADC_Start_DMA(
            adc_handle,
            (uint32_t *)adc_buffer,
            LINE_SENSOR_COUNT * LINE_OVERSAMPLE
        ) != HAL_OK)
    {
        adc_handle = NULL;
        return HAL_ERROR;
    }

    return HAL_OK;
}

/* =================================================
 * CAP NHAT DU LIEU 5 MAT
 * ================================================= */

void LineSensor_Update(void)
{
    int32_t weighted_sum = 0;
    uint32_t total_value = 0;

    if (adc_handle == NULL)
    {
        return;
    }

    /* -------------------------------------------------
     * 1. DAO THU TU ADC DMA SANG THU TU VAT LY
     * ------------------------------------------------- */

    {
        uint32_t sum[LINE_SENSOR_COUNT] = {0, 0, 0, 0, 0};

        /* Trung binh LINE_OVERSAMPLE lan quet cho tung kenh */
        for (uint32_t n = 0; n < LINE_OVERSAMPLE; n++)
        {
            for (uint32_t ch = 0; ch < LINE_SENSOR_COUNT; ch++)
            {
                sum[ch] += adc_buffer[(n * LINE_SENSOR_COUNT) + ch];
            }
        }

        sensor_data.raw[0] = (uint16_t)(sum[4] / LINE_OVERSAMPLE);  // S1 - PA4
        sensor_data.raw[1] = (uint16_t)(sum[3] / LINE_OVERSAMPLE);  // S2 - PA3
        sensor_data.raw[2] = (uint16_t)(sum[2] / LINE_OVERSAMPLE);  // S3 - PA2
        sensor_data.raw[3] = (uint16_t)(sum[1] / LINE_OVERSAMPLE);  // S4 - PA1
        sensor_data.raw[4] = (uint16_t)(sum[0] / LINE_OVERSAMPLE);  // S5 - PA0
    }

    /* -------------------------------------------------
     * 2. CHUAN HOA TUNG MAT
     * ------------------------------------------------- */

    for (uint8_t i = 0; i < LINE_SENSOR_COUNT; i++)
    {
        sensor_data.value[i] = LineSensor_Normalize(
            sensor_data.raw[i],
            black_calib[i],
            white_calib[i]
        );
    }

    /* -------------------------------------------------
     * 3. TINH TRUNG BINH CO TRONG SO
     * ------------------------------------------------- */

    {
        LinePreference_t pref = line_pref;

        if (pref == LINE_PREF_NONE)
        {
            /* Binh thuong: trung binh tat ca mat thay line */
            for (uint8_t i = 0; i < LINE_SENSOR_COUNT; i++)
            {
                uint32_t c = LineSensor_Contribution(sensor_data.value[i]);

                weighted_sum += (int32_t)weights[i] * (int32_t)c;

                total_value += c;
            }

            /* Khong mat nao vuot nguong phat hien -> mat line */
            {
                uint8_t seen = 0;

                for (uint8_t i = 0; i < LINE_SENSOR_COUNT; i++)
                {
                    if (sensor_data.value[i] >= LINE_DETECT_THRESHOLD)
                    {
                        seen = 1;
                    }
                }

                if (seen == 0)
                {
                    weighted_sum = 0;
                    total_value  = 0;
                }
            }
        }
        else
        {
            /* Chon nhom mat lien tiep (blob) o phia nhanh da chon */
            int8_t first = -1;
            int8_t last  = -1;
            int8_t start = -1;

            for (int8_t i = 0; i <= (int8_t)LINE_SENSOR_COUNT; i++)
            {
                uint8_t active =
                    (i < (int8_t)LINE_SENSOR_COUNT) &&
                    (sensor_data.value[i] >= LINE_DETECT_THRESHOLD);

                if (active && (start < 0))
                {
                    start = i;
                }
                else if ((!active) && (start >= 0))
                {
                    if (pref == LINE_PREF_LEFT)
                    {
                        if (first < 0)
                        {
                            first = start;
                            last  = i - 1;
                        }
                    }
                    else
                    {
                        first = start;      /* RIGHT: ghi de, giu nhom cuoi */
                        last  = i - 1;
                    }

                    start = -1;
                }
            }

            /* Hai line chua tach: vet den rong hon 3 mat.
               Chi lay 3 mat phia nhanh da chon. */
            if ((first >= 0) && ((last - first) > 2))
            {
                if (pref == LINE_PREF_LEFT)
                {
                    last = first + 2;
                }
                else
                {
                    first = last - 2;
                }
            }

            for (int8_t i = first; (first >= 0) && (i <= last); i++)
            {
                uint32_t c = LineSensor_Contribution(sensor_data.value[i]);

                weighted_sum += (int32_t)weights[i] * (int32_t)c;

                total_value += c;
            }
        }
    }

    /* -------------------------------------------------
     * 4. XAC DINH VI TRI LINE
     * ------------------------------------------------- */

    if (total_value == 0)
    {
        sensor_data.status = LINE_LOST;
        sensor_data.position = 0;
    }
    else
    {
        sensor_data.position =
            weighted_sum / (int32_t)total_value;

        sensor_data.status = LINE_DETECTED;
    }
}

/* =================================================
 * CHON NHANH BAM (NGA RE)
 * ================================================= */

void LineSensor_SetPreference(LinePreference_t pref)
{
    line_pref = pref;
}

/* =================================================
 * LAY DU LIEU CAM BIEN
 * ================================================= */

const LineSensor_Data_t *LineSensor_GetData(void)
{
    return &sensor_data;
}
