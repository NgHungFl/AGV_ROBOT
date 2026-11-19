
#include "load_detect.h"
#include "robot_config.h"

static TCS34725_ColorData_t sample;

static uint32_t baseline_x16 = 0;     /* baseline Clear * 16 */
static uint8_t  baseline_ok  = 0;

static uint32_t last_read_tick = 0;
static uint32_t present_since  = 0;
static uint8_t  present        = 0;

static uint32_t sum_r, sum_g, sum_b;
static uint16_t sum_n;

static LoadColor_t load_color = LOAD_COLOR_NONE;

HAL_StatusTypeDef LoadDetect_Init(I2C_HandleTypeDef *hi2c)
{
    baseline_x16 = 0;
    baseline_ok  = 0;

    LoadDetect_Reset();

    return TCS34725_Init(hi2c);
}

void LoadDetect_Reset(void)
{
    present        = 0;
    present_since  = 0;
    last_read_tick = HAL_GetTick();

    sum_r = 0;
    sum_g = 0;
    sum_b = 0;
    sum_n = 0;

    load_color = LOAD_COLOR_NONE;
}

uint16_t LoadDetect_GetBaseline(void)
{
    return (uint16_t)(baseline_x16 / 16U);
}

uint8_t LoadDetect_IsPresent(void)
{
    return present;
}

uint16_t LoadDetect_GetThreshold(void)
{
    return (uint16_t)((((baseline_x16 / 16U) * LOAD_PRESENT_PCT) / 100U) +
                      LOAD_PRESENT_DELTA);
}

const TCS34725_ColorData_t *LoadDetect_GetSample(void)
{
    return &sample;
}

LoadColor_t LoadDetect_GetColor(void)
{
    return load_color;
}

/* ================= BASELINE ================= */

void LoadDetect_UpdateBaseline(void)
{
    if ((HAL_GetTick() - last_read_tick) < LOAD_SAMPLE_PERIOD_MS)
    {
        return;
    }

    last_read_tick = HAL_GetTick();

    if (TCS34725_ReadColor(&sample) != HAL_OK)
    {
        return;
    }

    if (baseline_ok == 0)
    {
        baseline_x16 = (uint32_t)sample.clear * 16U;
        baseline_ok  = 1;
    }
    else
    {
        /* EMA: b += (x - b)/8  (dung so co dau, tranh tran so khong dau) */
        int32_t diff = ((int32_t)sample.clear * 16) - (int32_t)baseline_x16;

        baseline_x16 = (uint32_t)((int32_t)baseline_x16 + (diff / 8));
    }
}

/* ================= PHAN LOAI MAU ================= */

static LoadColor_t LoadDetect_Classify(uint32_t r, uint32_t b)
{
    if ((r * 10U) > (b * LOAD_COLOR_DOMINANCE_X10))
    {
        return LOAD_COLOR_RED;
    }

    if ((b * 10U) > (r * LOAD_COLOR_DOMINANCE_X10))
    {
        return LOAD_COLOR_BLUE;
    }

    return LOAD_COLOR_NONE;
}

/* ================= CHU KY CHO HANG ================= */

LoadStatus_t LoadDetect_Update(void)
{
    uint32_t now = HAL_GetTick();
    uint32_t threshold;
    uint32_t base;
    uint32_t diff;

    if (load_color != LOAD_COLOR_NONE)
    {
        return LOAD_STATUS_READY;
    }

    if ((now - last_read_tick) < LOAD_SAMPLE_PERIOD_MS)
    {
        return present ? LOAD_STATUS_CHECKING : LOAD_STATUS_EMPTY;
    }

    last_read_tick = now;

    if (TCS34725_ReadColor(&sample) != HAL_OK)
    {
        return present ? LOAD_STATUS_CHECKING : LOAD_STATUS_EMPTY;
    }

    base      = baseline_x16 / 16U;
    threshold = ((base * LOAD_PRESENT_PCT) / 100U) + LOAD_PRESENT_DELTA;

    /* Co hang khi Clear thay doi du lon so voi baseline (tang HOAC giam) */
    diff = ((uint32_t)sample.clear > base) ?
           ((uint32_t)sample.clear - base) :
           (base - (uint32_t)sample.clear);

    if (diff <= threshold)
    {
        /* Hang chua co hoac bi nhac ra */
        present = 0;
        sum_r = sum_g = sum_b = 0;
        sum_n = 0;

        return LOAD_STATUS_EMPTY;
    }

    if (present == 0)
    {
        present       = 1;
        present_since = now;
        sum_r = sum_g = sum_b = 0;
        sum_n = 0;
    }

    /* Bo 1/2 thoi gian xac nhan dau (tay con dang dat) roi moi lay mau */
    if ((now - present_since) >= (LOAD_CONFIRM_MS / 2U))
    {
        sum_r += sample.red;
        sum_g += sample.green;
        sum_b += sample.blue;
        sum_n++;
    }

    if (((now - present_since) >= LOAD_CONFIRM_MS) && (sum_n > 0U))
    {
        LoadColor_t c = LoadDetect_Classify(sum_r, sum_b);

        if (c != LOAD_COLOR_NONE)
        {
            load_color = c;
            return LOAD_STATUS_READY;
        }

        /* Chua phan biet duoc: lay mau lai */
        sum_r = sum_g = sum_b = 0;
        sum_n = 0;
        present_since = now - (LOAD_CONFIRM_MS / 2U);
    }

    return LOAD_STATUS_CHECKING;
}
