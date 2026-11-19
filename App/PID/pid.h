
#ifndef PID_H
#define PID_H

#include <stdint.h>

/* =================================================
 * PID DUNG CHUNG (vong toc do banh va vong bam line)
 *
 *   u = kp*e + I + kd*deriv
 *   I += ki*e*dt   (chong bao hoa: chi tich phan khi dau ra chua chan)
 *
 * deriv do NGUOI GOI cung cap (vd: toc do thay doi sai so),
 * nen vong bam line co the dua uoc luong tu bo quan sat Lyapunov vao.
 * ================================================= */

typedef struct
{
    float kp;
    float ki;
    float kd;

    float out_min;
    float out_max;
    float i_limit;

    float integral;
} PID_t;

void PID_Init(
    PID_t *pid,
    float kp,
    float ki,
    float kd,
    float out_min,
    float out_max,
    float i_limit
);

void PID_Reset(PID_t *pid);

void PID_SetLimits(PID_t *pid, float out_min, float out_max);

float PID_Update(PID_t *pid, float error, float deriv, float dt);

#endif /* PID_H */
