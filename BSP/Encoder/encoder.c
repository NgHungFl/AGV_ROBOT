
#include "encoder.h"
#include "robot_config.h"

extern TIM_HandleTypeDef htim1;   /* encoder banh trai  (PA8, PA9) */
extern TIM_HandleTypeDef htim4;   /* encoder banh phai  (PB6, PB7) */

static TIM_HandleTypeDef *const enc_tim[2] =
{
    &htim1,     /* MOTOR_LEFT  */
    &htim4      /* MOTOR_RIGHT */
};

static const int8_t enc_sign[2] =
{
    ENCODER_LEFT_SIGN,
    ENCODER_RIGHT_SIGN
};

static uint16_t last_cnt[2];
static int32_t  total_count[2];
static float    speed_mm_s[2];

/* ================= KHOI TAO ================= */

HAL_StatusTypeDef Encoder_Init(void)
{
    for (uint8_t i = 0; i < 2; i++)
    {
        if (HAL_TIM_Encoder_Start(enc_tim[i], TIM_CHANNEL_ALL) != HAL_OK)
        {
            return HAL_ERROR;
        }
    }

    Encoder_Reset();

    return HAL_OK;
}

void Encoder_Reset(void)
{
    for (uint8_t i = 0; i < 2; i++)
    {
        __HAL_TIM_SET_COUNTER(enc_tim[i], 0);

        last_cnt[i]    = 0;
        total_count[i] = 0;
        speed_mm_s[i]  = 0.0f;
    }
}

/* ================= CAP NHAT ================= */

void Encoder_Update(float dt)
{
    if (dt <= 0.0f)
    {
        return;
    }

    for (uint8_t i = 0; i < 2; i++)
    {
        uint16_t now = (uint16_t)__HAL_TIM_GET_COUNTER(enc_tim[i]);

        /* Bo dem 16 bit: phep tru roi ep kieu int16 xu ly tran so */
        int16_t delta = (int16_t)(uint16_t)(now - last_cnt[i]);

        last_cnt[i] = now;

        int32_t d = (int32_t)delta * enc_sign[i];

        total_count[i] += d;

        float raw = ((float)d * ROBOT_MM_PER_COUNT) / dt;

        speed_mm_s[i] += ENCODER_SPEED_ALPHA * (raw - speed_mm_s[i]);
    }
}

/* ================= LAY DU LIEU ================= */

int32_t Encoder_GetCount(Motor_ID_t motor)
{
    return total_count[motor];
}

float Encoder_GetDistance(Motor_ID_t motor)
{
    return (float)total_count[motor] * ROBOT_MM_PER_COUNT;
}

float Encoder_GetSpeed(Motor_ID_t motor)
{
    return speed_mm_s[motor];
}

float Encoder_GetCenterDistance(void)
{
    return 0.5f * (Encoder_GetDistance(MOTOR_LEFT) +
                   Encoder_GetDistance(MOTOR_RIGHT));
}

float Encoder_GetCenterSpeed(void)
{
    return 0.5f * (speed_mm_s[MOTOR_LEFT] + speed_mm_s[MOTOR_RIGHT]);
}

float Encoder_GetYawRate(void)
{
    /* Banh trai nhanh hon -> xe re phai -> duong */
    return (speed_mm_s[MOTOR_LEFT] - speed_mm_s[MOTOR_RIGHT]) /
           ROBOT_TRACK_WIDTH_MM;
}
