
#include "control_timer.h"

/* TIM2 duoc khoi tao thu cong (khong nam trong file .ioc)
   nen khong bi mat khi CubeMX tao lai code. */
static TIM_HandleTypeDef htim2;

static ControlTimer_Callback_t timer_callback = NULL;

HAL_StatusTypeDef ControlTimer_Init(
    ControlTimer_Callback_t callback,
    uint32_t period_ms
)
{
    if ((callback == NULL) || (period_ms == 0U) || (period_ms > 6500U))
    {
        return HAL_ERROR;
    }

    timer_callback = callback;

    __HAL_RCC_TIM2_CLK_ENABLE();

    /* Clock TIM2 = 72 MHz.
       Prescaler 7199 -> 10 kHz, moi tick = 0.1 ms */
    htim2.Instance               = TIM2;
    htim2.Init.Prescaler         = 7199;
    htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim2.Init.Period            = (period_ms * 10U) - 1U;
    htim2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
    {
        return HAL_ERROR;
    }

    /* Uu tien thap hon DMA ADC (0), cao hon SysTick (15) */
    HAL_NVIC_SetPriority(TIM2_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);

    return HAL_TIM_Base_Start_IT(&htim2);
}

void TIM2_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim2);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if ((htim->Instance == TIM2) && (timer_callback != NULL))
    {
        timer_callback();
    }
}
