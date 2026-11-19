
#include "encoder_test.h"
#include "encoder.h"
#include "motor.h"
#include "control_timer.h"
#include "robot_config.h"
#include "bsp_uart.h"

#define ENCODER_TEST_INTERVAL_MS 200U

static uint32_t last_print = 0;

/* Chay trong ngat TIM2 */
static void EncoderTest_Tick(void)
{
    Encoder_Update(ROBOT_CONTROL_DT_S);
}

/* =================================================
 * CACH TEST (khong cap dien dong co, xoay banh bang tay)
 *
 * 1. Xoay banh TRAI ve phia truoc 1 vong:
 *      CNT_L phai TANG ~ +1320, DIST_L ~ +100 mm
 *    Neu am  -> doi ENCODER_LEFT_SIGN thanh -1
 *    Neu CNT_L khong doi ma CNT_R doi -> TIM1/TIM4 dang bi nguoc trai/phai
 * 2. Lam tuong tu voi banh PHAI.
 * ================================================= */

void EncoderTest_Init(void)
{
    if (Motor_Init() != MOTOR_OK)
    {
        Error_Handler();
    }

    Motor_StopAll();

    if (Encoder_Init() != HAL_OK)
    {
        BSP_UART_Printf("ENCODER INIT FAILED\r\n");
        Error_Handler();
    }

    ControlTimer_Init(EncoderTest_Tick, ROBOT_CONTROL_PERIOD_MS);

    BSP_UART_Printf("ENCODER TEST: xoay tay tung banh ve phia truoc\r\n");

    last_print = HAL_GetTick();
}

void EncoderTest_Run(void)
{
    if ((HAL_GetTick() - last_print) < ENCODER_TEST_INTERVAL_MS)
    {
        return;
    }

    last_print = HAL_GetTick();

    BSP_UART_Printf(
        "CNT_L=%ld CNT_R=%ld | DIST_L=%ld DIST_R=%ld (mm) | V_L=%ld V_R=%ld (mm/s)\r\n",
        (long)Encoder_GetCount(MOTOR_LEFT),
        (long)Encoder_GetCount(MOTOR_RIGHT),
        (long)Encoder_GetDistance(MOTOR_LEFT),
        (long)Encoder_GetDistance(MOTOR_RIGHT),
        (long)Encoder_GetSpeed(MOTOR_LEFT),
        (long)Encoder_GetSpeed(MOTOR_RIGHT));
}
