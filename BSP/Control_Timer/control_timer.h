
#ifndef CONTROL_TIMER_H
#define CONTROL_TIMER_H

#include "main.h"

typedef void (*ControlTimer_Callback_t)(void);

/* TIM2: ngat moi period_ms, goi callback trong ngat */
HAL_StatusTypeDef ControlTimer_Init(
    ControlTimer_Callback_t callback,
    uint32_t period_ms
);

#endif /* CONTROL_TIMER_H */
