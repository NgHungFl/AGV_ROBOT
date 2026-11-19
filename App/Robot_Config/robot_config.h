
#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

/* =================================================
 * TOAN BO THONG SO CUA ROBOT NAM O DAY
 * Cac dong ghi "PHAI DO" la gia tri gia dinh,
 * can do thuc te roi sua lai.
 * ================================================= */

/* ================= CHU KY DIEU KHIEN ================= */

/* Vong dieu khien chay trong ngat TIM2 */
#define ROBOT_CONTROL_PERIOD_MS        10U
#define ROBOT_CONTROL_DT_S             0.010f

/* ================= CO KHI ================= */

#define ROBOT_WHEEL_CIRCUMFERENCE_MM   200.0f

/* Khoang cach giua 2 banh (tam banh - tam banh) - PHAI DO */
#define ROBOT_TRACK_WIDTH_MM           130.0f

/* Khoang cach tu truc banh den hang mat do line - PHAI DO */
#define ROBOT_SENSOR_DIST_MM           100.0f

/* Khoang cach giua 2 mat do line canh nhau */
#define ROBOT_SENSOR_PITCH_MM          15.0f

/* ================= ENCODER ================= */

#define ROBOT_ENC_PPR                  11.0f   /* xung / vong truc dong co */
#define ROBOT_ENC_MULT                 4.0f    /* doc 2 kenh, 2 canh = x4  */
#define ROBOT_GEAR_RATIO               30.0f

#define ROBOT_COUNTS_PER_REV \
    (ROBOT_ENC_PPR * ROBOT_ENC_MULT * ROBOT_GEAR_RATIO)      /* 1320 */

#define ROBOT_MM_PER_COUNT \
    (ROBOT_WHEEL_CIRCUMFERENCE_MM / ROBOT_COUNTS_PER_REV)    /* 0.1515 mm */

/* Chieu dem: xe tien thi so xung phai TANG. Neu am thi doi thanh -1 */
#define ENCODER_LEFT_SIGN              (1)
#define ENCODER_RIGHT_SIGN             (-1)

/* Loc toc do: 0 - 1. Cang nho cang muot nhung cang tre */
#define ENCODER_SPEED_ALPHA            0.4f

/* ================= MO HINH DONG CO (RIENG TUNG DONG CO) =================
 *
 * Quan he duty - toc do o che do xac lap:
 *      v = GAIN * (|duty| - DB)     khi |duty| > DB
 *      v = 0                        khi |duty| <= DB
 *   duty : % (0..100)      v : mm/s
 *   GAIN : mm/s tren 1 % duty      DB : vung chet (%)
 *
 * Do bang che do APP_MODE_DUTY_TEST roi dien so in ra vao day.
 * (de banh khong cham dat khi do) */

#define MOTOR_L_GAIN_FWD             11.98f
#define MOTOR_L_DB_FWD               4.89f
#define MOTOR_L_GAIN_REV             12.45f
#define MOTOR_L_DB_REV               7.35f
#define MOTOR_R_GAIN_FWD             12.62f
#define MOTOR_R_DB_FWD               5.06f
#define MOTOR_R_GAIN_REV             11.90f
#define MOTOR_R_DB_REV               3.55f

/* Neu duty DUONG ma banh quay LUI thi dat 1 (chi sua o day) */
#define MOTOR_LEFT_REVERSE             0
#define MOTOR_RIGHT_REVERSE            0

/* ================= VONG TOC DO BANH (PI, DAU RA = DUTY %) ================= */

/* Dau vao: sai so toc do (mm/s). Dau ra: duty hieu chinh (%), -100..100.
   duty cuoi = duty tu mo hinh dong co + duty cua PI */
#define WHEEL_KP                       0.436f      /* % tren mm/s */
#define WHEEL_KI                       7.5f      /* % tren (mm/s * s) */
#define WHEEL_INTEGRAL_LIMIT           40.0f       /* % */

/* ================= BO TAO QUY DAO VAN TOC / DUNG VI TRI ================= */

#define AGV_CRUISE_SPEED_MM_S          300.0f
#define AGV_ACCEL_MM_S2                400.0f
#define AGV_DECEL_MM_S2                500.0f

/* Phanh gap khi gap cuoi line */
#define AGV_END_DECEL_MM_S2            1500.0f

/* He so K_s cua luat vi tri: v = K_s * e (vung cuoi) */
#define AGV_POS_GAIN_KS                8.0f

/* Toc do toi thieu khi tiep can dich (thang ma sat) */
#define AGV_STOP_FLOOR_MM_S            45.0f

/* Sai so vi tri coi la da den dich */
#define AGV_STOP_TOL_MM                1.0f

/* Toc do khong duoc thap hon khi dang bam line (de bai: >= 0.1 m/s) */
#define AGV_MIN_TRACK_SPEED_MM_S       100.0f

/* Giam toc khi lech line lon */
#define AGV_CURVE_SLOW_GAIN            0.35f
#define AGV_CURVE_SLOW_REF_MM          25.0f

/* Mat line qua lau (ngoai vung cuoi) thi bao loi */
#define AGV_LOST_TIMEOUT_S             0.40f

/* ================= BO DIEU KHIEN BAM LINE: PID + LYAPUNOV =================
 *
 * Mot bo duy nhat. Ham Lyapunov V = y^2/2 + (L/2g)*b~^2 quyet dinh cau truc PID:
 *   P : Kp = K / L          (K : toc do hoi tu cua sai so ben, 1/s)
 *   D : Kd = C / L          (D lay tu goc lech theta uoc luong boi quan sat)
 *   I : Ki = GAMMA          (luat thich nghi bu nhieu hang so / lech 2 banh)
 * Khi C = 1 thi V_dot = -K*y^2 <= 0. */

#define LYAP_K                         8.0f
#define LYAP_C                         0.8f
#define LYAP_GAMMA                     0.010f      /* rad/s tren (mm*s) */
#define LYAP_I_LIMIT                   0.8f        /* rad/s */

/* Bo quan sat (y, theta): tan so rieng va he so tat dan */
#define OBS_WN                         25.0f
#define OBS_ZETA                       0.9f

/* Van toc nho nhat dung trong bo quan sat (tranh chia cho 0) */
#define OBS_V_MIN_MM_S                 60.0f

/* Mat line: giu omega cu trong thoi gian nay roi ve 0 */
#define LINE_LOST_HOLD_S               0.15f

/* Gioi han toc do quay (rad/s) */
#define LINE_OMEGA_MAX                 6.0f

/* ================= NHAN BIET HANG VA MAU ================= */

/* Co hang khi kenh Clear THAY DOI so voi baseline (tang HOAC giam):
      |Clear - baseline| > baseline * PCT / 100 + DELTA
   (baseline do luc chua co hang, xe dang chay toi khu tai hang) */
#define LOAD_PRESENT_PCT               35U
#define LOAD_PRESENT_DELTA             30U

/* Hang phai on dinh lien tuc thoi gian nay moi xac nhan */
#define LOAD_CONFIRM_MS                1000U

/* Chu ky doc cam bien (TCS34725 tich phan 50 ms) */
#define LOAD_SAMPLE_PERIOD_MS          60U

/* Phan biet mau: kenh troi hon kenh kia it nhat 1.3 lan */
#define LOAD_COLOR_DOMINANCE_X10       12U

/* ================= NHIEM VU ================= */

/* Quang duong tu luc xuat phat den diem dung tai hang (sa bo: 2000 mm) */
#define AGV_LOAD_ZONE_DIST_MM          2000.0f

/* Quang duong tu diem tai hang den cuoi line (ly thuyet = 4709 mm,
   sau do hieu chinh theo so do thuc te) */
#define AGV_END_PATH_LENGTH_MM         4709.0f

/* Vung cham truoc cuoi line */
#define AGV_END_SLOW_DIST_MM           250.0f
#define AGV_END_SLOW_SPEED_MM_S        100.0f

/* Neu qua dich them quang nay ma van chua thay cuoi line thi dung */
#define AGV_END_OVERRUN_MM             200.0f

/* Cho phep nhan biet cuoi line som hon dich nay (sai so odometry) */
#define AGV_END_DETECT_MARGIN_MM       200.0f

/* So mau lien tiep mat line moi xem la het line */
#define AGV_END_CONFIRM_SAMPLES        1U

/* Quang duong chay them sau khi phat hien het line */
#define AGV_END_EXTRA_MM               0.0f

/* Nhanh tren la DO (re trai), nhanh duoi la XANH (re phai).
   Neu sa ban nguoc lai thi doi 2 dong nay. */
#define AGV_BRANCH_RED                 LINE_PREF_LEFT
#define AGV_BRANCH_BLUE                LINE_PREF_RIGHT

#define AGV_START_DELAY_MS             3000U

/* Thoi gian cho nguoi rut tay ra sau khi nhan duoc hang */
#define AGV_LOAD_SETTLE_MS             1500U

#endif /* ROBOT_CONFIG_H */
