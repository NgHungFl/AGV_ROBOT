
#ifndef MOTOR_MODEL_H
#define MOTOR_MODEL_H

#include "motor.h"

/* Duty (%, co dau) can thiet de dong co dat toc do v (mm/s, co dau) o xac lap.
   Moi dong co, moi chieu quay co he so va vung chet rieng (robot_config.h). */
float MotorModel_DutyFromSpeed(Motor_ID_t motor, float v_mm_s);

/* Nguoc lai: toc do xac lap ung voi duty (%) */
float MotorModel_SpeedFromDuty(Motor_ID_t motor, float duty_pct);

#endif /* MOTOR_MODEL_H */
