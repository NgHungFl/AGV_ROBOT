
#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

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

void Motor_Stop(Motor_ID_t motor);

void Motor_StopAll(void);

void Motor_Disable(void);

#endif /* MOTOR_H */
