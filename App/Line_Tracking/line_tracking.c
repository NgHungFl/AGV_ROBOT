
#include "line_tracking.h"
#include "pid.h"
#include <math.h>

/* =================================================
 * LINE_LAW_STATE_FEEDBACK (Chuong 5):
 *   Doi dau e2, e3, w sang quy uoc duong ve PHAI cua firmware.
 *   Luu trang thai theo (eC_hat, e3_hat), eC_hat = e2_hat + l*e3_hat.
 *   Trong toa do nay, gain quan sat eC la L1 + l*L2 = 2*zeta*wn.
 *   Luat (5.27): w = -k2*(eC_hat - l*e3_hat) - k3*sin(e3_hat).
 *
 * LINE_LAW_KANAYAMA: bien the cu w = -k2*v*eC_hat - k3*sin(e3_hat).
 *
 * LINE_LAW_PID_LYAP: PID, CAU TRUC DO LYAPUNOV QUYET DINH
 *
 * Quy uoc: duong = ben PHAI.
 *   d     : vi tri line do duoc so voi tam hang mat (mm), d > 0 khi line ben phai
 *   y     : do lech cua hang mat so voi line = -d
 *   theta : goc lech giua huong xe va tiep tuyen line (rad), duong = lech phai
 *   v     : van toc tien (mm/s)
 *   w     : toc do quay phai (rad/s)  <- dau ra cua bo dieu khien
 *   L     : khoang cach tu truc banh den hang mat (mm)
 *   b     : nhieu hang so chua biet cong vao w (do cong line, 2 banh lech nhau)
 *
 *   y_dot     = v*theta + L*(w + b)
 *   theta_dot = w + b - v*kappa
 *
 * LUAT DIEU KHIEN (PID + Lyapunov)
 *   w = -(K*y + v*theta_hat)/L - b_hat ,   b_hat_dot = GAMMA * y
 *
 *   Viet theo d = -y:
 *   w = (K/L) * d              <- P  (Kp = K/L)
 *     + (C/L) * (-v*theta_hat) <- D  (Kd = C/L, -v*theta = toc do d_dot do goc lech)
 *     + GAMMA * Integral(d)    <- I  (Ki = GAMMA)
 *
 * CHUNG MINH (C = 1, theta do chinh xac):
 *   Thay vao: y_dot = -K*y + L*b~ ,  b~ = b - b_hat
 *   V = y^2/2 + (L / (2*GAMMA)) * b~^2
 *   V_dot = -K*y^2 + L*y*b~ - L*b~*y = -K*y^2 <= 0
 *   => y bi chan va y -> 0 (bo de Barbalat). Thanh phan I chinh la
 *   luat thich nghi, nen Ki = GAMMA khong chon tuy tien.
 *   Voi C < 1: sai so bi chan UUB, C cang gan 1 cang nho
 *   (chap nhan C < 1 de giam nhay nhieu cua uoc luong theta).
 *
 * Goc theta khong do truc tiep duoc -> bo quan sat Luenberger
 * dung omega do tu encoder:
 *   y_hat_dot     = v*theta_hat + L*w + l1*(y - y_hat)
 *   theta_hat_dot = w + l2*(y - y_hat)
 *   l1 = 2*zeta*wn ,  l2 = wn^2 / v
 * ================================================= */

#define LINE_L_MM        ROBOT_SENSOR_DIST_MM

static PID_t line_pid;

static float sensor_error_hat_mm;
static float theta_hat;
static uint8_t obs_ready;

static float error_mm;
static float speed_factor = 1.0f;
static float omega_cmd;
static float lost_time;

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

void LineTracking_Reset(void)
{
    PID_Init(
        &line_pid,
        LYAP_K / LINE_L_MM,         /* Kp */
        LYAP_GAMMA,                 /* Ki */
        LYAP_C / LINE_L_MM,         /* Kd */
        -LINE_OMEGA_MAX,
        LINE_OMEGA_MAX,
        LYAP_I_LIMIT);

    sensor_error_hat_mm = 0.0f;
    theta_hat = 0.0f;
    obs_ready = 0;

    error_mm  = 0.0f;
    omega_cmd = 0.0f;
    lost_time = 0.0f;

    speed_factor = 1.0f;
}

float LineTracking_GetSpeedFactor(void)
{
    return speed_factor;
}

uint8_t LineTracking_IsLost(void)
{
    return (lost_time > 0.0f) ? 1U : 0U;
}

float LineTracking_GetLostTime(void)
{
    return lost_time;
}

float LineTracking_GetError(void)
{
    return error_mm;
}

float LineTracking_GetHeadingEst(void)
{
    return theta_hat;
}

float LineTracking_GetAxleErrorEst(void)
{
    return sensor_error_hat_mm - (LINE_L_MM * theta_hat);
}

float LineTracking_Compute(float dt, float v_mm_s, float omega_meas)
{
    const LineSensor_Data_t *data = LineSensor_GetData();

    const float v_eff = (v_mm_s > OBS_V_MIN_MM_S) ? v_mm_s : OBS_V_MIN_MM_S;

    float y_meas;
    float innov;
#if (LINE_LAW == LINE_LAW_STATE_FEEDBACK)
    const float v_obs = (v_mm_s > 0.0f) ? v_mm_s : 0.0f;
    const float wn = OBS_STATE_WN;
    /* Giam hieu chinh goc lien tuc ve 0 khi xe dung (det O = v_M). */
    const float heading_scale = v_obs / v_eff;
#else
    const float v_obs = v_eff;
    const float wn = OBS_WN;
    const float heading_scale = 1.0f;
#endif
    const float sensor_gain = 2.0f * OBS_ZETA * wn;
    const float heading_gain = ((wn * wn) / v_eff) * heading_scale;

    /* ================= MAT LINE ================= */

    if (data->status == LINE_LOST)
    {
        lost_time += dt;

        /* Quan sat chi du doan, khong co do luong */
        sensor_error_hat_mm += dt * ((v_obs * theta_hat) + (LINE_L_MM * omega_meas));
        theta_hat += dt * omega_meas;

        if (lost_time > LINE_LOST_HOLD_S)
        {
            omega_cmd = 0.0f;
#if (LINE_LAW == LINE_LAW_STATE_FEEDBACK)
            obs_ready = 0;
#endif
        }

        return omega_cmd;
    }

    lost_time = 0.0f;

    /* d: vi tri line (mm), duong = ben phai */
    error_mm = LINE_POSITION_TO_MM(data->position_filtered);
    y_meas   = -error_mm;

    /* ================= BO QUAN SAT ================= */

    if (obs_ready == 0)
    {
        sensor_error_hat_mm = y_meas;
        theta_hat = 0.0f;
        obs_ready = 1;
    }

    innov = Limit(y_meas - sensor_error_hat_mm, OBS_INNOV_MAX_MM);

    sensor_error_hat_mm += dt * ((v_obs * theta_hat) + (LINE_L_MM * omega_meas) +
                                (sensor_gain * innov));
    theta_hat += dt * (omega_meas + (heading_gain * innov));

    theta_hat = Limit(theta_hat, OBS_THETA_MAX_RAD);

#if (LINE_LAW == LINE_LAW_STATE_FEEDBACK)

    omega_cmd = Limit(
        -(LINE_STATE_K2 * LineTracking_GetAxleErrorEst()) - (LINE_STATE_K3 * sinf(theta_hat)),
        LINE_OMEGA_MAX);
    speed_factor = 1.0f;

#elif (LINE_LAW == LINE_LAW_KANAYAMA)

    omega_cmd = Limit(
        (LINE_K2 * (-sensor_error_hat_mm) * v_eff) - (LINE_K3 * sinf(theta_hat)),
        LINE_OMEGA_MAX);

    speed_factor = cosf(theta_hat);

#else

    /* ================= PID (cau truc Lyapunov) =================
       error = d_hat = -sensor_error_hat_mm
       deriv = -v*theta_hat : toc do thay doi cua d do goc lech xe */

    omega_cmd = PID_Update(
        &line_pid,
        -sensor_error_hat_mm,
        -v_eff * theta_hat,
        dt);

    speed_factor = 1.0f;

#endif

    return omega_cmd;
}
