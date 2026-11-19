
#include "pid.h"

void PID_Init(
    PID_t *pid,
    float kp,
    float ki,
    float kd,
    float out_min,
    float out_max,
    float i_limit)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->out_min = out_min;
    pid->out_max = out_max;
    pid->i_limit = i_limit;

    pid->integral = 0.0f;
}

void PID_Reset(PID_t *pid)
{
    pid->integral = 0.0f;
}

void PID_SetLimits(PID_t *pid, float out_min, float out_max)
{
    pid->out_min = out_min;
    pid->out_max = out_max;
}

float PID_Update(PID_t *pid, float error, float deriv, float dt)
{
    float u = (pid->kp * error) + pid->integral + (pid->kd * deriv);

    /* Anti-windup: chi tich phan khi chua bao hoa,
       hoac khi sai so keo dau ra ve phia khong bao hoa */
    if (((u < pid->out_max) && (u > pid->out_min)) ||
        ((u >= pid->out_max) && (error < 0.0f)) ||
        ((u <= pid->out_min) && (error > 0.0f)))
    {
        pid->integral += pid->ki * error * dt;

        if (pid->integral > pid->i_limit)
        {
            pid->integral = pid->i_limit;
        }
        else if (pid->integral < -pid->i_limit)
        {
            pid->integral = -pid->i_limit;
        }
    }

    u = (pid->kp * error) + pid->integral + (pid->kd * deriv);

    if (u > pid->out_max)
    {
        u = pid->out_max;
    }
    else if (u < pid->out_min)
    {
        u = pid->out_min;
    }

    return u;
}
