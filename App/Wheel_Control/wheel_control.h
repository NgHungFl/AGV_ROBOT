
#ifndef WHEEL_CONTROL_H
#define WHEEL_CONTROL_H

#include "main.h"
#include "motor.h"

void WheelControl_Init(void);

/* Dat toc do mong muon 2 banh (mm/s), am = lui */
void WheelControl_SetRef(float left_mm_s, float right_mm_s);

/* Phanh va giu xe (tat PID). SetRef se thoat che do nay */
void WheelControl_Hold(void);

/* Goi moi chu ky dieu khien, sau Encoder_Update() */
void WheelControl_Update(float dt);

float WheelControl_GetRef(Motor_ID_t motor);

/* Duty cuoi cung gui xuong dong co (%, co dau, -100..100) */
float WheelControl_GetDuty(Motor_ID_t motor);

#endif /* WHEEL_CONTROL_H */
