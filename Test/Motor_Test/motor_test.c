
#include "motor_test.h"
#include "motor.h"
#include "main.h"

void MotorTest_Run(void)
{
    /* Test 1: Left motor forward */
    Motor_SetSpeed(MOTOR_LEFT, 25);
    HAL_Delay(1500);

    Motor_Stop(MOTOR_LEFT);
    HAL_Delay(1000);

    /* Test 2: Left motor reverse */
    Motor_SetSpeed(MOTOR_LEFT, -25);
    HAL_Delay(1500);

    Motor_Stop(MOTOR_LEFT);
    HAL_Delay(1000);

    /* Test 3: Right motor forward */
    Motor_SetSpeed(MOTOR_RIGHT, 25);
    HAL_Delay(1500);

    Motor_Stop(MOTOR_RIGHT);
    HAL_Delay(1000);

    /* Test 4: Right motor reverse */
    Motor_SetSpeed(MOTOR_RIGHT, -25);
    HAL_Delay(1500);

    Motor_Stop(MOTOR_RIGHT);
    HAL_Delay(1000);

    /* Test 5: Both motors forward */
    Motor_SetSpeed(MOTOR_LEFT, 25);
    Motor_SetSpeed(MOTOR_RIGHT, 25);
    HAL_Delay(2000);

    Motor_StopAll();
    HAL_Delay(1000);

    /* Test 6: Both motors reverse */
    Motor_SetSpeed(MOTOR_LEFT, -25);
    Motor_SetSpeed(MOTOR_RIGHT, -25);
    HAL_Delay(2000);

    Motor_StopAll();

    /* Finish test and disable driver */
    Motor_Disable();
}
