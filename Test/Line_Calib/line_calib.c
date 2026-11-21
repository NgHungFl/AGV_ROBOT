
#include "line_calib.h"
#include "line_sensor.h"
#include "bsp_uart.h"

#include "main.h"

/* =================================================
 * TU DONG DO BLACK / WHITE CHO 5 MAT
 *
 * Giai doan 1 (3 s)  : dat 5 mat tren NEN TRANG, giu yen
 *                      -> WHITE = trung binh, do luon nhieu (max - min)
 * Giai doan 2 (8 s)  : truot xe ngang qua LINE DEN (qua lai nhieu lan,
 *                      de MOI mat deu di qua giua line)
 *                      -> BLACK = gia tri xa WHITE nhat
 * Cuoi cung in ra dang #define de dan vao line_sensor.h
 * ================================================= */

#define CALIB_WHITE_MS   3000U
#define CALIB_BLACK_MS   8000U

/* Tuong phan toi thieu (so muc ADC) va ty le tuong phan / nhieu */
#define CALIB_MIN_SPAN        300U
#define CALIB_MIN_SNR         10U

static uint32_t t_start;
static uint32_t t_print;
static uint8_t  phase;

static uint32_t white_sum[LINE_SENSOR_COUNT];
static uint32_t white_n;
static uint16_t white_min[LINE_SENSOR_COUNT];
static uint16_t white_max[LINE_SENSOR_COUNT];
static uint16_t white_avg[LINE_SENSOR_COUNT];
static uint16_t black_val[LINE_SENSOR_COUNT];

void LineCalib_Init(void)
{
    for (uint8_t i = 0; i < LINE_SENSOR_COUNT; i++)
    {
        white_sum[i] = 0;
        white_min[i] = 0xFFFFU;
        white_max[i] = 0;
        white_avg[i] = 0;
        black_val[i] = 0;
    }

    white_n = 0;
    phase   = 0;

    BSP_UART_Printf("\r\nLINE CALIBRATION\r\n");
    BSP_UART_Printf("B1 (3 s): DAT 5 MAT TREN NEN TRANG, GIU YEN\r\n");

    t_start = HAL_GetTick();
    t_print = t_start;
}

static void LineCalib_PrintResult(void)
{
    BSP_UART_Printf("\r\n---- KET QUA: dan vao line_sensor.h ----\r\n");

    for (uint8_t i = 0; i < LINE_SENSOR_COUNT; i++)
    {
        uint16_t span = (black_val[i] > white_avg[i]) ?
                        (uint16_t)(black_val[i] - white_avg[i]) :
                        (uint16_t)(white_avg[i] - black_val[i]);

        uint16_t noise = (uint16_t)(white_max[i] - white_min[i]);

        BSP_UART_Printf("#define LINE_S%u_BLACK %uU\r\n",
                        (unsigned)(i + 1U), (unsigned)black_val[i]);
        BSP_UART_Printf("#define LINE_S%u_WHITE %uU\r\n",
                        (unsigned)(i + 1U), (unsigned)white_avg[i]);

        BSP_UART_Printf("   S%u: tuong phan=%u muc, nhieu nen trang=%u muc\r\n",
                        (unsigned)(i + 1U), (unsigned)span, (unsigned)noise);

        if (span < CALIB_MIN_SPAN)
        {
            BSP_UART_Printf("   !! S%u TUONG PHAN THAP (< %u muc ADC): doi dien tro "
                            "phat/thu hoac ha thap khoang cach mat - mat dat\r\n",
                            (unsigned)(i + 1U), (unsigned)CALIB_MIN_SPAN);
        }
        else if ((noise > 0U) && (span < (CALIB_MIN_SNR * noise)))
        {
            BSP_UART_Printf("   !! S%u NHIEU CAO so voi tuong phan\r\n",
                            (unsigned)(i + 1U));
        }
    }

    BSP_UART_Printf("---- XONG ----\r\n");
}

void LineCalib_Run(void)
{
    const LineSensor_Data_t *d;
    uint32_t t;

    if (phase >= 2U)
    {
        return;
    }

    LineSensor_Update();
    d = LineSensor_GetData();

    t = HAL_GetTick() - t_start;

    if (phase == 0U)
    {
        for (uint8_t i = 0; i < LINE_SENSOR_COUNT; i++)
        {
            white_sum[i] += d->raw[i];

            if (d->raw[i] < white_min[i]) { white_min[i] = d->raw[i]; }
            if (d->raw[i] > white_max[i]) { white_max[i] = d->raw[i]; }
        }

        white_n++;

        if (t >= CALIB_WHITE_MS)
        {
            for (uint8_t i = 0; i < LINE_SENSOR_COUNT; i++)
            {
                white_avg[i] = (uint16_t)(white_sum[i] / white_n);
                black_val[i] = white_avg[i];
            }

            phase   = 1;
            t_start = HAL_GetTick();

            BSP_UART_Printf("B2 (8 s): TRUOT XE NGANG QUA LINE DEN, QUA LAI "
                            "NHIEU LAN (moi mat deu qua giua line)\r\n");
        }

        return;
    }

    /* phase 1: tim gia tri xa WHITE nhat */
    for (uint8_t i = 0; i < LINE_SENSOR_COUNT; i++)
    {
        int32_t cur = (int32_t)d->raw[i] - (int32_t)white_avg[i];
        int32_t best = (int32_t)black_val[i] - (int32_t)white_avg[i];

        if (cur < 0)  { cur = -cur; }
        if (best < 0) { best = -best; }

        if (cur > best)
        {
            black_val[i] = d->raw[i];
        }
    }

    if ((HAL_GetTick() - t_print) >= 1000U)
    {
        t_print = HAL_GetTick();

        BSP_UART_Printf("  raw: %u %u %u %u %u\r\n",
                        (unsigned)d->raw[0], (unsigned)d->raw[1],
                        (unsigned)d->raw[2], (unsigned)d->raw[3],
                        (unsigned)d->raw[4]);
    }

    if (t >= CALIB_BLACK_MS)
    {
        phase = 2;
        LineCalib_PrintResult();
    }
}
