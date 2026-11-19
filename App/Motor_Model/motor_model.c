
#include "motor_model.h"
#include "robot_config.h"

typedef struct
{
    float gain_fwd;     /* mm/s tren 1 % */
    float db_fwd;       /* %             */
    float gain_rev;
    float db_rev;
} MotorModel_t;

static const MotorModel_t model[2] =
{
    /* MOTOR_LEFT */
    { MOTOR_L_GAIN_FWD, MOTOR_L_DB_FWD, MOTOR_L_GAIN_REV, MOTOR_L_DB_REV },

    /* MOTOR_RIGHT */
    { MOTOR_R_GAIN_FWD, MOTOR_R_DB_FWD, MOTOR_R_GAIN_REV, MOTOR_R_DB_REV }
};

float MotorModel_DutyFromSpeed(Motor_ID_t motor, float v_mm_s)
{
    const MotorModel_t *m = &model[motor];

    if (v_mm_s > 1.0f)
    {
        return (v_mm_s / m->gain_fwd) + m->db_fwd;
    }

    if (v_mm_s < -1.0f)
    {
        return (v_mm_s / m->gain_rev) - m->db_rev;
    }

    return 0.0f;
}

float MotorModel_SpeedFromDuty(Motor_ID_t motor, float duty_pct)
{
    const MotorModel_t *m = &model[motor];

    if (duty_pct > m->db_fwd)
    {
        return m->gain_fwd * (duty_pct - m->db_fwd);
    }

    if (duty_pct < -m->db_rev)
    {
        return m->gain_rev * (duty_pct + m->db_rev);
    }

    return 0.0f;
}
