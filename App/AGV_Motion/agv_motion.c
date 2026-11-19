
#include "agv_motion.h"
#include "robot_config.h"
#include "encoder.h"
#include "wheel_control.h"
#include "motion_profile.h"
#include "line_tracking.h"
#include <math.h>

/* ================= TRANG THAI ================= */

static volatile AGV_MotionState_t motion_state = AGV_MOTION_IDLE;

static AGV_Segment_t seg;

static float s_origin   = 0.0f;
static float s_now      = 0.0f;
static float v_ref      = 0.0f;
static float omega_cmd  = 0.0f;
static float end_s      = 0.0f;

static uint8_t end_latched  = 0;
static uint8_t lost_samples = 0;

/* ================= HAM NOI BO ================= */

static float Limit(float x, float lim)
{
    if (x > lim)
    {
        return lim;
    }

    if (x < -lim)
    {
        return -lim;
    }

    return x;
}

/* ================= KHOI TAO ================= */

void AGV_Motion_Init(void)
{
    motion_state = AGV_MOTION_IDLE;

    LineTracking_Reset();
    WheelControl_Init();
}

void AGV_Motion_SetBranch(LinePreference_t pref)
{
    LineSensor_SetPreference(pref);
}

void AGV_Motion_Start(const AGV_Segment_t *segment)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();

    seg = *segment;

    s_origin = Encoder_GetCenterDistance();

    LineTracking_Reset();

    if (seg.stop_on_line_end)
    {
        MotionProfile_Start(seg.target_mm + seg.overrun_mm, seg.cruise_mm_s);
    }
    else
    {
        MotionProfile_Start(seg.target_mm, seg.cruise_mm_s);
    }

    end_latched  = 0;
    lost_samples = 0;
    end_s        = 0.0f;
    v_ref        = 0.0f;
    omega_cmd    = 0.0f;

    motion_state = AGV_MOTION_RUNNING;

    __set_PRIMASK(primask);
}

/* ================= DOAN DANG CHAY ================= */

static void AGV_Motion_Run(float dt)
{
    const LineSensor_Data_t *line = LineSensor_GetData();

    float v_meas = Encoder_GetCenterSpeed();
    float w_meas = Encoder_GetYawRate();

    float v_cmd;
    float w_lim;
    float k_slow;
    float zone = seg.target_mm - seg.slow_dist_mm;

    s_now = Encoder_GetCenterDistance() - s_origin;

    /* ---------- 1. PHAT HIEN CUOI LINE ---------- */

    if ((seg.stop_on_line_end != 0) && (end_latched == 0))
    {
        MotionProfile_SetCruise(
            (s_now >= zone) ? seg.slow_speed_mm_s : seg.cruise_mm_s);

        if (line->status == LINE_LOST)
        {
            if (lost_samples < 255U)
            {
                lost_samples++;
            }
        }
        else
        {
            lost_samples = 0;
        }

        if ((s_now >= (zone - AGV_END_DETECT_MARGIN_MM)) &&
            (lost_samples >= AGV_END_CONFIRM_SAMPLES))
        {
            end_latched = 1;
            end_s       = s_now;

            MotionProfile_SetDecel(AGV_END_DECEL_MM_S2);
            MotionProfile_SetTarget(s_now + AGV_END_EXTRA_MM);
        }
    }

    /* ---------- 2. QUY DAO VAN TOC / DUNG VI TRI ---------- */

    v_ref = MotionProfile_Update(s_now, dt);

    if (MotionProfile_IsDone())
    {
        WheelControl_Hold();

        if (end_latched)
        {
            motion_state = AGV_MOTION_LINE_END;
        }
        else if (seg.stop_on_line_end)
        {
            motion_state = AGV_MOTION_FAULT;   /* qua dich ma khong thay cuoi line */
        }
        else
        {
            motion_state = AGV_MOTION_ARRIVED;
        }

        return;
    }

    /* ---------- 3. BAM LINE ---------- */

    if ((end_latched != 0) || (seg.line_follow == 0))
    {
        omega_cmd = 0.0f;
    }
    else
    {
        omega_cmd = LineTracking_Compute(dt, v_meas, w_meas);

        /* Mat line qua lau ngoai vung cuoi -> loi */
        if ((LineTracking_GetLostTime() > AGV_LOST_TIMEOUT_S) &&
            ((seg.stop_on_line_end == 0) ||
             (s_now < (zone - AGV_END_DETECT_MARGIN_MM))))
        {
            WheelControl_Hold();
            motion_state = AGV_MOTION_FAULT;
            return;
        }
    }

    /* ---------- 4. GIAM TOC KHI LECH LINE ---------- */

    k_slow = fabsf(LineTracking_GetError()) / AGV_CURVE_SLOW_REF_MM;

    if (k_slow > 1.0f)
    {
        k_slow = 1.0f;
    }

    k_slow = 1.0f - (AGV_CURVE_SLOW_GAIN * k_slow);

    v_cmd = v_ref * k_slow;

    if (v_cmd < AGV_MIN_TRACK_SPEED_MM_S)
    {
        v_cmd = (v_ref < AGV_MIN_TRACK_SPEED_MM_S) ? v_ref : AGV_MIN_TRACK_SPEED_MM_S;
    }

    /* ---------- 5. TOC DO 2 BANH ---------- */

    /* Banh trong khong dao chieu: |w| * W/2 <= v */
    w_lim = (2.0f * v_cmd) / ROBOT_TRACK_WIDTH_MM;

    omega_cmd = Limit(omega_cmd, w_lim);

    WheelControl_SetRef(
        v_cmd + (omega_cmd * ROBOT_TRACK_WIDTH_MM * 0.5f),
        v_cmd - (omega_cmd * ROBOT_TRACK_WIDTH_MM * 0.5f));
}

/* ================= NGAT DIEU KHIEN ================= */

void AGV_Motion_Tick(void)
{
    Encoder_Update(ROBOT_CONTROL_DT_S);

    LineSensor_Update();

    if (motion_state == AGV_MOTION_RUNNING)
    {
        AGV_Motion_Run(ROBOT_CONTROL_DT_S);
    }
    else
    {
        WheelControl_Hold();
    }

    WheelControl_Update(ROBOT_CONTROL_DT_S);
}

/* ================= DU LIEU ================= */

AGV_MotionState_t AGV_Motion_GetState(void)
{
    return motion_state;
}

float AGV_Motion_GetDistance(void)
{
    return s_now;
}

float AGV_Motion_GetSpeedRef(void)
{
    return v_ref;
}

float AGV_Motion_GetOmegaCmd(void)
{
    return omega_cmd;
}

float AGV_Motion_GetEndDistance(void)
{
    return end_s;
}
