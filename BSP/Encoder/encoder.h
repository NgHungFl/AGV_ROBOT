
#ifndef ENCODER_H
#define ENCODER_H

#include "main.h"
#include "motor.h"
#include <stdint.h>

/* Khoi tao 2 encoder (TIM1 = trai, TIM4 = phai) */
HAL_StatusTypeDef Encoder_Init(void);

/* Dat lai bo dem ve 0 */
void Encoder_Reset(void);

/* Goi dung chu ky ROBOT_CONTROL_PERIOD_MS (trong ngat TIM2) */
void Encoder_Update(float dt);

/* So xung tich luy (da nhan dau) */
int32_t Encoder_GetCount(Motor_ID_t motor);

/* Quang duong tung banh (mm) */
float Encoder_GetDistance(Motor_ID_t motor);

/* Toc do tung banh (mm/s), da loc */
float Encoder_GetSpeed(Motor_ID_t motor);

/* Quang duong tam xe (mm) = trung binh 2 banh */
float Encoder_GetCenterDistance(void);

/* Van toc tien cua tam xe (mm/s) */
float Encoder_GetCenterSpeed(void);

/* Toc do quay (rad/s), DUONG = quay PHAI */
float Encoder_GetYawRate(void);

#endif /* ENCODER_H */
