
#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

#define MOTOR_DUTY_MAX 3599

/* Motor identification */
typedef enum
{
    MOTOR_LEFT = 0,
    MOTOR_RIGHT
} Motor_ID_t;

/* Function status */
typedef enum
{
    MOTOR_OK = 0,
    MOTOR_ERROR
} Motor_Status_t;

/* Motor API */
Motor_Status_t Motor_Init(void);

Motor_Status_t Motor_SetSpeed(Motor_ID_t motor, int16_t speed);

/* Dat PWM truc tiep: -3599 .. +3599 (phan giai min hon SetSpeed 36 lan) */
Motor_Status_t Motor_SetDuty(Motor_ID_t motor, int16_t duty);

/* Dat duty theo %, co dau: -100.0 .. +100.0 */
Motor_Status_t Motor_SetDutyPercent(Motor_ID_t motor, float duty_pct);

/* Phanh dong nang (TB6612: IN1 = IN2 = 1), giu xe dung yen */
void Motor_Brake(Motor_ID_t motor);

void Motor_BrakeAll(void);

void Motor_Stop(Motor_ID_t motor);

void Motor_StopAll(void);

void Motor_Disable(void);

#endif /* MOTOR_H */
