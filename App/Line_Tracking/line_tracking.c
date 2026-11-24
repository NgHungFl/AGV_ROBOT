
#include "line_tracking.h"
#include "pid.h"
#include <math.h>

/* =================================================
 * LINE_LAW_KANAYAMA (mac dinh, trung voi bao cao mo hinh hoa):
 *
 *   Mo hinh (5.11):  e2_dot = v*sin(e3) - w*(l + e1)
 *                    e3_dot = w_R - w
 *   Luat (5.13):     w = k2*e2*v + w_R + k3*sin(e3)
 *   Ham Lyapunov:    V = (e1^2 + e2^2)/2 + (1 - cos e3)/k2
 *   V_dot = -k1*e1^2 - (k3/k2)*sin^2(e3) - w*l*e2
 *   Luu y: so hang -w*l*e2 khong xac dinh dau; bao cao bo qua no (l nho)
 *   => on dinh o muc xap xi. Khi e1 = 0, V_dot chi nua xac dinh am nen
 *   e2 -> 0 phai suy ra tu LaSalle / Barbalat voi v_R != 0.
 *
 *   Doi dau sang quy uoc code (duong = ben PHAI, w quay phai):
 *      e2 = -d ,  e3 = theta ,  w_code = k2*d*v - k3*sin(theta)
 *   e3 lay tu bo quan sat (co tinh den so hang l*w) thay cho (5.17).
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

static float y_hat;
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

    y_hat     = 0.0f;
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

float LineTracking_Compute(float dt, float v_mm_s, float omega_meas)
{
    const LineSensor_Data_t *data = LineSensor_GetData();

    float v_eff = (v_mm_s > OBS_V_MIN_MM_S) ? v_mm_s : OBS_V_MIN_MM_S;

    float y_meas;
    float innov;
    float l1 = 2.0f * OBS_ZETA * OBS_WN;
    float l2 = (OBS_WN * OBS_WN) / v_eff;

    /* ================= MAT LINE ================= */

    if (data->status == LINE_LOST)
    {
        lost_time += dt;

        /* Quan sat chi du doan, khong co do luong */
        y_hat     += dt * ((v_eff * theta_hat) + (LINE_L_MM * omega_meas));
        theta_hat += dt * omega_meas;

        if (lost_time > LINE_LOST_HOLD_S)
        {
            omega_cmd = 0.0f;
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
        y_hat     = y_meas;
        theta_hat = 0.0f;
        obs_ready = 1;
    }

    innov = Limit(y_meas - y_hat, OBS_INNOV_MAX_MM);

    y_hat     += dt * ((v_eff * theta_hat) + (LINE_L_MM * omega_meas) + (l1 * innov));
    theta_hat += dt * (omega_meas + (l2 * innov));

    theta_hat = Limit(theta_hat, OBS_THETA_MAX_RAD);

#if (LINE_LAW == LINE_LAW_KANAYAMA)

    /* ================= LUAT (5.13) =================
       w = k2*e2*v + k3*sin(e3)  (e2 = -d_hat, e3 = theta_hat; dau doi sang
       quy uoc quay phai).  v = v_R*cos(e3). */

    omega_cmd = Limit(
        (LINE_K2 * (-y_hat) * v_eff) - (LINE_K3 * sinf(theta_hat)),
        LINE_OMEGA_MAX);

    speed_factor = cosf(theta_hat);

#else

    /* ================= PID (cau truc Lyapunov) =================
       error = d_hat = -y_hat
       deriv = -v*theta_hat : toc do thay doi cua d do goc lech xe */

    omega_cmd = PID_Update(
        &line_pid,
        -y_hat,
        -v_eff * theta_hat,
        dt);

    speed_factor = 1.0f;

#endif

    return omega_cmd;
}
