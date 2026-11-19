
#include "wheel_control.h"
#include "encoder.h"
#include "motor_model.h"
#include "pid.h"
#include "robot_config.h"

static PID_t   speed_pid[2];
static float   ref_speed[2];
static float   duty_out[2];
static uint8_t hold_mode = 1;

void WheelControl_Init(void)
{
    for (uint8_t i = 0; i < 2; i++)
    {
        PID_Init(
            &speed_pid[i],
            WHEEL_KP,
            WHEEL_KI,
            0.0f,
            -100.0f,
            100.0f,
            WHEEL_INTEGRAL_LIMIT);

        ref_speed[i] = 0.0f;
        duty_out[i]  = 0.0f;
    }

    hold_mode = 1;
}

void WheelControl_SetRef(float left_mm_s, float right_mm_s)
{
    ref_speed[MOTOR_LEFT]  = left_mm_s;
    ref_speed[MOTOR_RIGHT] = right_mm_s;

    hold_mode = 0;
}

void WheelControl_Hold(void)
{
    hold_mode = 1;

    for (uint8_t i = 0; i < 2; i++)
    {
        ref_speed[i] = 0.0f;
        PID_Reset(&speed_pid[i]);
    }
}

float WheelControl_GetRef(Motor_ID_t motor)
{
    return ref_speed[motor];
}

float WheelControl_GetDuty(Motor_ID_t motor)
{
    return duty_out[motor];
}

/* =================================================
 * duty = duty_model(ref) + PID(ref - v_do)
 *
 * duty_model : mo hinh rieng tung dong co (motor_model.c)
 * PID        : bu sai lech mo hinh / tai / pin yeu
 * Gioi han PID dong theo duty_model de tong luon nam trong -100..100.
 * ================================================= */

void WheelControl_Update(float dt)
{
    if (hold_mode)
    {
        Motor_BrakeAll();

        duty_out[MOTOR_LEFT]  = 0.0f;
        duty_out[MOTOR_RIGHT] = 0.0f;

        return;
    }

    for (uint8_t i = 0; i < 2; i++)
    {
        float ff   = MotorModel_DutyFromSpeed((Motor_ID_t)i, ref_speed[i]);
        float err  = ref_speed[i] - Encoder_GetSpeed((Motor_ID_t)i);
        float trim;

        PID_SetLimits(&speed_pid[i], -100.0f - ff, 100.0f - ff);

        trim = PID_Update(&speed_pid[i], err, 0.0f, dt);

        duty_out[i] = ff + trim;

        Motor_SetDutyPercent((Motor_ID_t)i, duty_out[i]);
    }
}
