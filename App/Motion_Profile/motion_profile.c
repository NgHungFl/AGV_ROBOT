
#include "motion_profile.h"
#include "robot_config.h"
#include <math.h>

/* =================================================
 * LUAT DUNG VI TRI THEO LYAPUNOV
 *
 * Sai so vi tri: e = target - s        (de bai: e -> 0)
 * Dong hoc    : e_dot = -v
 * Ham Lyapunov: V = e^2 / 2   (V > 0 khi e != 0)
 *
 * Chon v = K_s * e        ->  V_dot = -K_s * e^2          < 0
 * Chon v = sqrt(2*a*e)    ->  V_dot = -sqrt(2a) * e^1.5   < 0
 *
 * => e hoi tu ve 0 (tiem can / huu han).
 * Duong sqrt(2*a*e) chinh la duong phanh co gia toc khong
 * doi bang a. Gan dich, luat tuyen tinh K_s*e tranh dao dong
 * (sqrt co do doc vo han tai e = 0).
 * Van toc luon <= cruise. Co san toc do toi thieu de thang ma sat.
 * ================================================= */

static float   target_pos  = 0.0f;
static float   cruise_spd  = 0.0f;
static float   decel       = AGV_DECEL_MM_S2;
static float   v_ref       = 0.0f;
static uint8_t done        = 1;

void MotionProfile_Start(float target_mm, float cruise_mm_s)
{
    target_pos = target_mm;
    cruise_spd = cruise_mm_s;
    decel      = AGV_DECEL_MM_S2;
    v_ref      = 0.0f;
    done       = 0;
}

void MotionProfile_SetTarget(float target_mm)
{
    target_pos = target_mm;
}

void MotionProfile_SetCruise(float cruise_mm_s)
{
    cruise_spd = cruise_mm_s;
}

void MotionProfile_SetDecel(float decel_mm_s2)
{
    decel = decel_mm_s2;
}

uint8_t MotionProfile_IsDone(void)
{
    return done;
}

float MotionProfile_GetTarget(void)
{
    return target_pos;
}

float MotionProfile_Update(float s, float dt)
{
    float e = target_pos - s;
    float v_des;
    float dv;

    if (done)
    {
        return 0.0f;
    }

    /* Da den dich */
    if (e <= AGV_STOP_TOL_MM)
    {
        v_ref = 0.0f;
        done  = 1;
        return 0.0f;
    }

    /* Van toc mong muon theo luat Lyapunov */
    v_des = sqrtf(2.0f * decel * e);

    if (v_des > (AGV_POS_GAIN_KS * e))
    {
        v_des = AGV_POS_GAIN_KS * e;
    }

    if (v_des > cruise_spd)
    {
        v_des = cruise_spd;
    }

    if (v_des < AGV_STOP_FLOOR_MM_S)
    {
        v_des = AGV_STOP_FLOOR_MM_S;
    }

    /* Gioi han gia toc cua van toc mong muon */
    dv = v_des - v_ref;

    if (dv > (AGV_ACCEL_MM_S2 * dt))
    {
        dv = AGV_ACCEL_MM_S2 * dt;
    }
    else if (dv < -(1.5f * decel * dt))
    {
        dv = -(1.5f * decel * dt);
    }

    v_ref += dv;

    return v_ref;
}
