
#include "line_tracking.h"
#include "line_sensor.h"

/* ================= BIEN PID ================= */

static float integral = 0.0f;
static int32_t previous_error = 0;
static uint32_t previous_time = 0;

/* ================= BIEN NHAN DIEN VACH NGANG ================= */

static uint32_t all_black_start = 0;
static uint8_t all_black_timer_active = 0;
static uint8_t finish_detected = 0;

/* ================= CAC HAM NOI BO ================= */

static int16_t LimitSpeed(int16_t speed)
{
    if (speed > 100)
        speed = 100;

    if (speed < -100)
        speed = -100;

    return speed;
}


static float LimitIntegral(float value)
{
    if (value > LINE_INTEGRAL_LIMIT)
        value = LINE_INTEGRAL_LIMIT;

    if (value < -LINE_INTEGRAL_LIMIT)
        value = -LINE_INTEGRAL_LIMIT;

    return value;
}


static float LimitCorrection(float value)
{
    if (value > LINE_MAX_CORRECTION)
        value = LINE_MAX_CORRECTION;

    if (value < -LINE_MAX_CORRECTION)
        value = -LINE_MAX_CORRECTION;

    return value;
}


/* ================= KHOI TAO ================= */

Motor_Status_t LineTracking_Init(void)
{
    /* Reset PID */
    integral = 0.0f;
    previous_error = 0;
    previous_time = HAL_GetTick();

    /* Reset nhan dien vach ket thuc */
    all_black_start = 0;
    all_black_timer_active = 0;
    finish_detected = 0;

    /* Khoi tao dong co */
    Motor_StopAll();

    if (Motor_Init() != MOTOR_OK)
    {
        return MOTOR_ERROR;
    }

    return MOTOR_OK;
}


/* ================= DUNG XE ================= */

void LineTracking_Stop(void)
{
    Motor_StopAll();

    integral = 0.0f;
    previous_error = 0;
}


/* ================= CHUONG TRINH PID BAM LINE ================= */

void LineTracking_Run(void)
{
    uint32_t current_time;
    uint32_t elapsed_time;

    float dt;
    float p_term;
    float i_term;
    float d_term;
    float output;

    int32_t error;

    int16_t correction;
    int16_t left_speed;
    int16_t right_speed;

    const LineSensor_Data_t *data;

    uint8_t all_black = 1;

    /* Xe da nhan dien vach ket thuc thi giu dung */
    if (finish_detected)
    {
        Motor_StopAll();
        return;
    }

    /* Chi dieu khien moi 10 ms */
    current_time = HAL_GetTick();
    elapsed_time = current_time - previous_time;

    if (elapsed_time < LINE_CONTROL_PERIOD)
    {
        return;
    }

    previous_time = current_time;

    dt = (float)elapsed_time / 1000.0f;

    if (dt <= 0.0f)
    {
        return;
    }

    /* Cap nhat du lieu 5 cam bien */
    LineSensor_Update();

    data = LineSensor_GetData();

    /* ================= KIEM TRA CA 5 MAT DEU DEN ================= */

#if LINE_ALL_BLACK_STOP_ENABLE

    for (uint8_t i = 0; i < LINE_SENSOR_COUNT; i++)
    {
        if (data->value[i] < LINE_ALL_BLACK_THRESHOLD)
        {
            all_black = 0;
            break;
        }
    }

    if (all_black)
    {
        if (all_black_timer_active == 0)
        {
            all_black_start = current_time;
            all_black_timer_active = 1;
        }

        if ((current_time - all_black_start) >= LINE_ALL_BLACK_TIME)
        {
            finish_detected = 1;

            Motor_StopAll();

            integral = 0.0f;
            previous_error = 0;

            return;
        }
    }
    else
    {
        all_black_timer_active = 0;
        all_black_start = 0;
    }

#endif

    /* ================= KIEM TRA MAT LINE ================= */

    if (data->status == LINE_LOST)
    {
        Motor_StopAll();

        integral = 0.0f;
        previous_error = 0;

        return;
    }

    /* ================= TINH PID ================= */

    error = data->position;

    /* P */
    p_term = LINE_KP * (float)error;

    /* I */
    integral += (float)error * dt;
    integral = LimitIntegral(integral);

    i_term = LINE_KI * integral;

    /* D */
    d_term = LINE_KD *
             ((float)(error - previous_error) / dt);

    previous_error = error;

    /* Tong PID */
    output = p_term + i_term + d_term;

    /* Gioi han muc dieu chinh */
    output = LimitCorrection(output);

    correction = (int16_t)output;

    /* ================= DIEU KHIEN 2 DONG CO ================= */

    left_speed  = LINE_BASE_SPEED + correction;
    right_speed = LINE_BASE_SPEED - correction;

    left_speed  = LimitSpeed(left_speed);
    right_speed = LimitSpeed(right_speed);

    Motor_SetSpeed(MOTOR_LEFT, left_speed);
    Motor_SetSpeed(MOTOR_RIGHT, right_speed);
}
