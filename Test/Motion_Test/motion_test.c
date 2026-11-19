
#include "motion_test.h"
#include "motor.h"
#include "encoder.h"
#include "wheel_control.h"
#include "agv_motion.h"
#include "line_tracking.h"
#include "control_timer.h"
#include "robot_config.h"
#include "bsp_uart.h"

#define TEST_PRINT_MS   20U

static MotionTest_t test_id;
static uint32_t     t0;
static uint32_t     last_print;
static uint8_t      finished;

/* ================= TICK CHO TEST VONG TOC DO ================= */

static void SpeedTest_Tick(void)
{
    Encoder_Update(ROBOT_CONTROL_DT_S);
    WheelControl_Update(ROBOT_CONTROL_DT_S);
}

static void DutyTest_Tick(void)
{
    Encoder_Update(ROBOT_CONTROL_DT_S);
}

/* =================================================
 * DO MO HINH DONG CO:  v = GAIN * (|duty| - DB)
 *
 * Moi dong co, moi chieu: quet duty 10..80 %, moi nac 1.2 s,
 * lay trung binh toc do o 0.5 s cuoi. Sau do hoi quy tuyen tinh
 * tren cac diem co toc do > 30 mm/s.
 * Ket qua in ra dang #define de dan thang vao robot_config.h
 * ================================================= */

#define DUTY_POINTS      8U
#define DUTY_STEP_PCT    10.0f
#define DUTY_SETTLE_MS   700U
#define DUTY_MEASURE_MS  500U

static float    duty_v[4][DUTY_POINTS];
static uint8_t  duty_combo = 0;
static uint8_t  duty_point = 0;
static uint32_t duty_t_step = 0;
static float    duty_acc = 0.0f;
static uint32_t duty_n = 0;

static void DutyTest_Fit(uint8_t combo)
{
    float sx = 0, sy = 0, sxx = 0, sxy = 0;
    uint8_t n = 0;

    for (uint8_t i = 0; i < DUTY_POINTS; i++)
    {
        float u = DUTY_STEP_PCT * (float)(i + 1U);
        float v = duty_v[combo][i];

        if (v > 30.0f)
        {
            sx += u;  sy += v;  sxx += u * u;  sxy += u * v;
            n++;
        }
    }

    if (n >= 2U)
    {
        float den = ((float)n * sxx) - (sx * sx);
        float k   = (((float)n * sxy) - (sx * sy)) / den;
        float b   = (sy - (k * sx)) / (float)n;
        float db  = (k > 0.0f) ? (-b / k) : 0.0f;

        BSP_UART_Printf("#define MOTOR_%c_GAIN_%s   %ld.%02ldf\r\n",
                        (combo < 2U) ? 'L' : 'R',
                        ((combo & 1U) == 0U) ? "FWD" : "REV",
                        (long)k, (long)((k - (long)k) * 100.0f));

        BSP_UART_Printf("#define MOTOR_%c_DB_%s     %ld.%02ldf\r\n",
                        (combo < 2U) ? 'L' : 'R',
                        ((combo & 1U) == 0U) ? "FWD" : "REV",
                        (long)db, (long)((db - (long)db) * 100.0f));
    }
    else
    {
        BSP_UART_Printf("Motor %u: KHONG DU DIEM (toc do am hoac = 0?) -> "
                        "kiem tra MOTOR_xxx_REVERSE / ENCODER_xxx_SIGN\r\n",
                        (unsigned)combo);
    }
}

static void DutyTest_Run(void)
{
    uint32_t now = HAL_GetTick();
    uint32_t dt  = now - duty_t_step;

    Motor_ID_t m   = (duty_combo < 2U) ? MOTOR_LEFT : MOTOR_RIGHT;
    float      dir = ((duty_combo & 1U) == 0U) ? 1.0f : -1.0f;
    float      u   = DUTY_STEP_PCT * (float)(duty_point + 1U);

    if (dt < DUTY_SETTLE_MS)
    {
        Motor_SetDutyPercent(m, dir * u);
        return;
    }

    if (dt < (DUTY_SETTLE_MS + DUTY_MEASURE_MS))
    {
        duty_acc += dir * Encoder_GetSpeed(m);
        duty_n++;
        return;
    }

    /* Het nac */
    duty_v[duty_combo][duty_point] = (duty_n > 0U) ? (duty_acc / (float)duty_n) : 0.0f;

    BSP_UART_Printf("%c%c duty=%ld%%  v=%ld mm/s\r\n",
                    (m == MOTOR_LEFT) ? 'L' : 'R',
                    (dir > 0.0f) ? '+' : '-',
                    (long)u,
                    (long)duty_v[duty_combo][duty_point]);

    duty_acc = 0.0f;
    duty_n = 0;
    duty_t_step = HAL_GetTick();
    duty_point++;

    if (duty_point >= DUTY_POINTS)
    {
        Motor_Stop(m);
        HAL_Delay(600);

        duty_point = 0;
        duty_combo++;
        duty_t_step = HAL_GetTick();

        if (duty_combo >= 4U)
        {
            BSP_UART_Printf("---- DAN VAO robot_config.h ----\r\n");

            for (uint8_t c = 0; c < 4U; c++)
            {
                DutyTest_Fit(c);
            }

            finished = 1;
        }
    }
}

/* ================= KHOI TAO ================= */

void MotionTest_Init(MotionTest_t test)
{
    static const AGV_Segment_t seg_straight =
    {
        .target_mm = 1000.0f, .cruise_mm_s = 250.0f, .line_follow = 0,
        .stop_on_line_end = 0, .slow_dist_mm = 0.0f,
        .slow_speed_mm_s = 0.0f, .overrun_mm = 0.0f
    };

    static const AGV_Segment_t seg_follow =
    {
        .target_mm = 3000.0f, .cruise_mm_s = AGV_CRUISE_SPEED_MM_S, .line_follow = 1,
        .stop_on_line_end = 0, .slow_dist_mm = 0.0f,
        .slow_speed_mm_s = 0.0f, .overrun_mm = 0.0f
    };

    test_id  = test;
    finished = 0;

    if (Motor_Init() != MOTOR_OK)
    {
        Error_Handler();
    }

    if (Encoder_Init() != HAL_OK)
    {
        Error_Handler();
    }

    WheelControl_Init();
    AGV_Motion_Init();

    if (test == MOTION_TEST_DUTY)
    {
        ControlTimer_Init(DutyTest_Tick, ROBOT_CONTROL_PERIOD_MS);
        BSP_UART_Printf("DUTY TEST: nhac banh khoi san! Bat dau sau 3 s\r\n");
    }
    else if (test == MOTION_TEST_SPEED)
    {
        ControlTimer_Init(SpeedTest_Tick, ROBOT_CONTROL_PERIOD_MS);
        BSP_UART_Printf("t_ms,ref,vL,vR,duL_x10,duR_x10\r\n");
    }
    else
    {
        ControlTimer_Init(AGV_Motion_Tick, ROBOT_CONTROL_PERIOD_MS);
    }

    HAL_Delay((test == MOTION_TEST_DUTY) ? 3000U : 2000U);

    duty_t_step = HAL_GetTick();

    t0         = HAL_GetTick();
    last_print = t0;

    if (test == MOTION_TEST_STRAIGHT)
    {
        AGV_Motion_Start(&seg_straight);
    }
    else if (test == MOTION_TEST_FOLLOW)
    {
        AGV_Motion_Start(&seg_follow);
        BSP_UART_Printf("t_ms,s,vref,v,d10,w100,th1000\r\n");
    }
}

/* ================= CHAY TEST ================= */

void MotionTest_Run(void)
{
    uint32_t t = HAL_GetTick() - t0;

    if (finished)
    {
        return;
    }

    /* ---------- DO MO HINH DONG CO ---------- */

    if (test_id == MOTION_TEST_DUTY)
    {
        DutyTest_Run();
        return;
    }

    /* ---------- NAC TOC DO ---------- */

    if (test_id == MOTION_TEST_SPEED)
    {
        float ref = 0.0f;

        if      (t < 500U)   { ref = 0.0f;   }
        else if (t < 2500U)  { ref = 150.0f; }
        else if (t < 4500U)  { ref = 300.0f; }
        else if (t < 6000U)  { ref = 100.0f; }
        else
        {
            WheelControl_Hold();
            BSP_UART_Printf("SPEED TEST DONE\r\n");
            finished = 1;
            return;
        }

        WheelControl_SetRef(ref, ref);

        if ((HAL_GetTick() - last_print) >= TEST_PRINT_MS)
        {
            last_print = HAL_GetTick();

            BSP_UART_Printf("%lu,%ld,%ld,%ld,%ld,%ld\r\n",
                            (unsigned long)t,
                            (long)ref,
                            (long)Encoder_GetSpeed(MOTOR_LEFT),
                            (long)Encoder_GetSpeed(MOTOR_RIGHT),
                            (long)(WheelControl_GetDuty(MOTOR_LEFT) * 10.0f),
                            (long)(WheelControl_GetDuty(MOTOR_RIGHT) * 10.0f));
        }

        return;
    }

    /* ---------- CHAY THANG / BAM LINE ---------- */

    if ((HAL_GetTick() - last_print) >= 50U)
    {
        last_print = HAL_GetTick();

        BSP_UART_Printf("%lu,%ld,%ld,%ld,%ld,%ld,%ld\r\n",
                        (unsigned long)t,
                        (long)AGV_Motion_GetDistance(),
                        (long)AGV_Motion_GetSpeedRef(),
                        (long)Encoder_GetCenterSpeed(),
                        (long)(LineTracking_GetError() * 10.0f),
                        (long)(AGV_Motion_GetOmegaCmd() * 100.0f),
                        (long)(LineTracking_GetHeadingEst() * 1000.0f));
    }

    if (AGV_Motion_GetState() != AGV_MOTION_RUNNING)
    {
        HAL_Delay(500);

        BSP_UART_Printf(
            "TEST DONE: S=%ld mm, CNT_L=%ld, CNT_R=%ld, state=%d\r\n",
            (long)AGV_Motion_GetDistance(),
            (long)Encoder_GetCount(MOTOR_LEFT),
            (long)Encoder_GetCount(MOTOR_RIGHT),
            (int)AGV_Motion_GetState());

        finished = 1;
    }
}
