#include "line_auto_calib.h"
#include "line_calib_storage.h"
#include "line_sensor.h"
#include "robot_config.h"
#include "bsp_uart.h"
#include <stdint.h>

/*
 * Two-surface calibration, no sweeping:
 * WHITE (all 5 sensors on real white floor, stationary, running height)
 * -> BLACK (slide a wide real black patch beneath all 5, stationary)
 * -> validate 5 contrasts -> save once to Flash.
 *
 * During calibration, PWM/motor/control timer has NOT been started by main.
 */

#define CALIB_LED_WHITE_BLINK_MS 350U
#define CALIB_LED_BLACK_BLINK_MS 120U

static void CalibLed_Set(uint8_t on)
{
    /* PC13 on common Blue Pill boards is active-low. */
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin,
                      on ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

static void CalibLed_Pulse(uint8_t times)
{
    for (uint8_t i = 0U; i < times; i++)
    {
        CalibLed_Set(1U); HAL_Delay(100U);
        CalibLed_Set(0U); HAL_Delay(130U);
    }
}

static void Calib_PrintValues(const char *title,
                              const uint16_t white[LINE_SENSOR_COUNT],
                              const uint16_t black[LINE_SENSOR_COUNT])
{
    BSP_UART_Printf("%s\r\n", title);
    for (uint8_t i = 0U; i < LINE_SENSOR_COUNT; i++)
    {
        BSP_UART_Printf("S%u: WHITE=%u BLACK=%u SPAN=%ld\r\n",
                        (unsigned)(i + 1U),
                        (unsigned)white[i], (unsigned)black[i],
                        (long)((int32_t)black[i]-(int32_t)white[i]));
    }
}

static uint8_t Calib_ReadRaw(uint16_t raw[LINE_SENSOR_COUNT])
{
    LineSensor_Update();
    const LineSensor_Data_t *data = LineSensor_GetData();
    for (uint8_t i = 0U; i < LINE_SENSOR_COUNT; i++)
    {
        raw[i] = data->raw[i];
        if ((raw[i] == 0U) || (raw[i] >= 4095U)) { return 0U; }
    }
    return 1U;
}

/* Collect a contiguous *stable* time window for ALL five sensors.
 * For BLACK, require a minimum contrast against the measured WHITE on every
 * sample; this rejects partial coverage and transitions in which only one
 * or two eyes see the black patch.
 *
 * An unstable/out-of-range sample restarts the current candidate window;
 * it does not cancel the calibration until the phase times out.
 */
static HAL_StatusTypeDef Calib_ReadStableSurface(
    uint8_t need_black,
    const uint16_t white[LINE_SENSOR_COUNT],
    uint16_t out[LINE_SENSOR_COUNT],
    uint32_t timeout_ms,
    uint32_t blink_ms)
{
    uint32_t sum[LINE_SENSOR_COUNT] = {0U};
    uint16_t lo[LINE_SENSOR_COUNT];
    uint16_t hi[LINE_SENSOR_COUNT];
    uint16_t good = 0U;
    uint32_t last_sample;
    uint32_t last_report;
    uint32_t start = HAL_GetTick();

    for (uint8_t i = 0U; i < LINE_SENSOR_COUNT; i++)
    {
        lo[i] = 4095U;
        hi[i] = 0U;
    }
    last_sample = start - LINE_AUTO_CALIB_SAMPLE_MS;
    last_report = start;

    while ((HAL_GetTick() - start) < timeout_ms)
    {
        uint32_t now = HAL_GetTick();
        uint16_t raw[LINE_SENSOR_COUNT];
        uint8_t valid = 1U;

        CalibLed_Set((((now - start) / blink_ms) % 2U) == 0U);
        if ((now - last_sample) < LINE_AUTO_CALIB_SAMPLE_MS)
        {
            HAL_Delay(1U);
            continue;
        }
        last_sample = now;
        if (!Calib_ReadRaw(raw))
        {
            good = 0U;
            continue;
        }

        if (need_black)
        {
            for (uint8_t i = 0U; i < LINE_SENSOR_COUNT; i++)
            {
                /* ADC black > ADC white on the measured AGV hardware. */
                if ((int32_t)raw[i] - (int32_t)white[i] <
                    (int32_t)LINE_AUTO_CALIB_MIN_SPAN)
                {
                    valid = 0U;
                    break;
                }
            }
        }

        if (valid)
        {
            for (uint8_t i = 0U; i < LINE_SENSOR_COUNT; i++)
            {
                uint16_t mn = (raw[i] < lo[i]) ? raw[i] : lo[i];
                uint16_t mx = (raw[i] > hi[i]) ? raw[i] : hi[i];
                if ((uint16_t)(mx - mn) > LINE_AUTO_CALIB_STABLE_RANGE)
                {
                    valid = 0U;
                    break;
                }
            }
        }

        if (!valid)
        {
            good = 0U;
            for (uint8_t i = 0U; i < LINE_SENSOR_COUNT; i++)
            {
                sum[i] = 0U;
                lo[i] = 4095U;
                hi[i] = 0U;
            }
            /* When the transition ends, the next stationary values start
             * a completely new 400ms window. */
            continue;
        }

        for (uint8_t i = 0U; i < LINE_SENSOR_COUNT; i++)
        {
            if (raw[i] < lo[i]) { lo[i] = raw[i]; }
            if (raw[i] > hi[i]) { hi[i] = raw[i]; }
            sum[i] += raw[i];
        }
        good++;

        if (good >= LINE_AUTO_CALIB_STABLE_SAMPLES)
        {
            for (uint8_t i = 0U; i < LINE_SENSOR_COUNT; i++)
            {
                out[i] = (uint16_t)((sum[i] + good/2U) / good);
            }
            CalibLed_Set(0U);
            BSP_UART_Printf("CALIB %s: stable %u samples, %lu ms\r\n",
                            need_black ? "BLACK" : "WHITE",
                            (unsigned)good,
                            (unsigned long)(HAL_GetTick()-start));
            return HAL_OK;
        }

        if ((now - last_report) > 1000U)
        {
            last_report = now;
            BSP_UART_Printf("CALIB %s: waiting for stable surface (%u/%u)\r\n",
                            need_black ? "BLACK" : "WHITE",
                            (unsigned)good,
                            (unsigned)LINE_AUTO_CALIB_STABLE_SAMPLES);
        }
    }

    CalibLed_Set(0U);
    BSP_UART_Printf("CALIB ERROR: %s stable-window timeout (%lu ms)\r\n",
                    need_black ? "BLACK" : "WHITE",
                    (unsigned long)timeout_ms);
    return HAL_ERROR;
}

HAL_StatusTypeDef LineAutoCalib_Boot(uint8_t manual_nrst)
{
    uint16_t white[LINE_SENSOR_COUNT];
    uint16_t black[LINE_SENSOR_COUNT];
    uint8_t have_saved = LineCalibStorage_Load(white, black);

    CalibLed_Set(0U);
    if (!manual_nrst && have_saved)
    {
        if (LineSensor_SetCalibration(white, black) != HAL_OK) { return HAL_ERROR; }
        BSP_UART_Printf("LINE CALIB: loaded from FLASH (NO SCAN)\r\n");
        Calib_PrintValues("CURRENT CALIBRATION:", white, black);
        CalibLed_Pulse(2U);
        return HAL_OK;
    }

    BSP_UART_Printf("BOOT: %s -> TWO-SURFACE AUTO CALIB\r\n",
                    manual_nrst ? "NRST pressed" : "no valid Flash data");
    BSP_UART_Printf("PHASE 1/2: 5 sensors on WHITE at running height; PC13 blinks SLOW\r\n");
    HAL_Delay(150U); /* let circular ADC DMA settle */
    if (Calib_ReadStableSurface(0U, white, white,
                               LINE_AUTO_CALIB_WHITE_TIMEOUT_MS,
                               CALIB_LED_WHITE_BLINK_MS) != HAL_OK)
    {
        BSP_UART_Printf("CALIB FAILED: old Flash kept; press NRST to retry\r\n");
        return HAL_ERROR;
    }

    for (uint8_t i = 0U; i < LINE_SENSOR_COUNT; i++)
    {
        BSP_UART_Printf("WHITE S%u=%u\r\n", (unsigned)(i+1U), (unsigned)white[i]);
    }
    CalibLed_Pulse(2U); /* distinctive transition before FAST blinking */
    BSP_UART_Printf("PHASE 2/2: slide WIDE BLACK patch under ALL 5 sensors; keep still; PC13 blinks FAST\r\n");
    if (Calib_ReadStableSurface(1U, white, black,
                               LINE_AUTO_CALIB_BLACK_TIMEOUT_MS,
                               CALIB_LED_BLACK_BLINK_MS) != HAL_OK)
    {
        BSP_UART_Printf("CALIB FAILED: old Flash kept; press NRST to retry\r\n");
        return HAL_ERROR;
    }

    Calib_PrintValues("CALIB MEASURED:", white, black);
    if (LineSensor_SetCalibration(white, black) != HAL_OK)
    {
        BSP_UART_Printf("CALIB ERROR: measured contrast not valid\r\n");
        return HAL_ERROR;
    }
    if (LineCalibStorage_Save(white, black) != HAL_OK)
    {
        BSP_UART_Printf("CALIB ERROR: Flash write/verify failed\r\n");
        return HAL_ERROR;
    }

    BSP_UART_Printf("CALIB SAVED IN FLASH - READY\r\n");
    CalibLed_Set(1U); HAL_Delay(900U); CalibLed_Set(0U);
    return HAL_OK;
}

void LineAutoCalib_ErrorBlink(void)
{
    HAL_GPIO_WritePin(ENBM_GPIO_Port, ENBM_Pin, GPIO_PIN_RESET);
    while (1)
    {
        CalibLed_Set(1U); HAL_Delay(100U);
        CalibLed_Set(0U); HAL_Delay(100U);
    }
}
