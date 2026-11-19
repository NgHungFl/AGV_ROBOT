#pragma once
#include <stdint.h>
#include <stddef.h>
typedef enum {HAL_OK=0,HAL_ERROR} HAL_StatusTypeDef;
typedef struct {uint32_t CNT;} TIM_TypeDef;
typedef struct {TIM_TypeDef *Instance;} TIM_HandleTypeDef;
typedef struct {int d;} ADC_HandleTypeDef;
typedef struct {int d;} I2C_HandleTypeDef;
typedef struct {int d;} UART_HandleTypeDef;
#define TIM_CHANNEL_ALL 0
uint32_t HAL_GetTick(void);
HAL_StatusTypeDef HAL_TIM_Encoder_Start(TIM_HandleTypeDef*,uint32_t);
#define __HAL_TIM_GET_COUNTER(h) ((h)->Instance->CNT)
#define __HAL_TIM_SET_COUNTER(h,v) ((h)->Instance->CNT=(v))
HAL_StatusTypeDef HAL_ADCEx_Calibration_Start(ADC_HandleTypeDef*);
HAL_StatusTypeDef HAL_ADC_Start_DMA(ADC_HandleTypeDef*,uint32_t*,uint32_t);
#define __get_PRIMASK() 0u
#define __set_PRIMASK(x) ((void)(x))
#define __disable_irq() ((void)0)
