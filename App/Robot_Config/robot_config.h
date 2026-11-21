
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
#define ROBOT_TRACK_WIDTH_MM           210.0f

/* l: khoang cach tu trung diem 2 banh (M) den tam day mat (C). Bao cao: 45.5 mm */
#define ROBOT_SENSOR_DIST_MM           45.5f

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

/* ================= MO HINH DONG CO (THEO BAO CAO MO HINH HOA) =================
 *
 * Ham truyen nhan dang (MATLAB System Identification), RIENG TUNG DONG CO:
 *      G(s) = K / (tau*s + 1)       K : rpm tren 1 % duty
 * Doi sang mm/s:  v[mm/s] = rpm * chu_vi / 60                                */

#define ROBOT_MM_S_PER_RPM             (ROBOT_WHEEL_CIRCUMFERENCE_MM / 60.0f)

#define MOTOR_L_K_RPM_PER_PCT          3.5168f
#define MOTOR_L_TAU_S                  0.0950f
#define MOTOR_R_K_RPM_PER_PCT          3.7137f
#define MOTOR_R_TAU_S                  0.0875f

/* Quan he xac lap dung cho feedforward:
 *      v = GAIN * (|duty| - DB)   khi |duty| > DB
 * Bao cao chua tinh vung chet nen DB = 0. Neu APP_MODE_DUTY_TEST do ra
 * DB > 0 thi dien vao day (feedforward chinh xac hon). */
#define MOTOR_L_GAIN_FWD               (MOTOR_L_K_RPM_PER_PCT * ROBOT_MM_S_PER_RPM)
#define MOTOR_L_DB_FWD                 0.0f
#define MOTOR_L_GAIN_REV               (MOTOR_L_K_RPM_PER_PCT * ROBOT_MM_S_PER_RPM)
#define MOTOR_L_DB_REV                 0.0f

#define MOTOR_R_GAIN_FWD               (MOTOR_R_K_RPM_PER_PCT * ROBOT_MM_S_PER_RPM)
#define MOTOR_R_DB_FWD                 0.0f
#define MOTOR_R_GAIN_REV               (MOTOR_R_K_RPM_PER_PCT * ROBOT_MM_S_PER_RPM)
#define MOTOR_R_DB_REV                 0.0f

/* Neu duty DUONG ma banh quay LUI thi dat 1 (chi sua o day) */
#define MOTOR_LEFT_REVERSE             0
#define MOTOR_RIGHT_REVERSE            0

/* ================= VONG TOC DO BANH: PI THEO BAO CAO =================
 *
 * Gain thiet ke o don vi cua bao cao: sai so [rpm] -> duty [%].
 * Tu dong doi sang mm/s:  K[% tren mm/s] = K[% tren rpm] / MM_S_PER_RPM
 *
 * PID Tuner (Ts ~ 0.19 s, vot lo 0 %)  <- dang dung
 * Tinh tay  (Ts ~ 0.195 s, vot lo ~8 %): L: 0.7961 / 17.76,  R: 0.67 / 15.49 */

#define WHEEL_L_KP_RPM                 0.50643f
#define WHEEL_L_KI_RPM                 5.7877f
#define WHEEL_R_KP_RPM                 0.50218f
#define WHEEL_R_KI_RPM                 5.2861f

#define WHEEL_INTEGRAL_LIMIT           40.0f       /* % */

/* 1: duty = mo hinh(v_ref) + PI   (nhanh hon, dung mo hinh G rieng tung dong co)
   0: chi PI nhu bao cao */
#define WHEEL_USE_FEEDFORWARD          1

/* ================= BO TAO QUY DAO VAN TOC / DUNG VI TRI ================= */

#define AGV_CRUISE_SPEED_MM_S          500.0f
#define AGV_ACCEL_MM_S2                200.0f
#define AGV_DECEL_MM_S2                200.0f

/* Phanh gap khi gap cuoi line */
#define AGV_END_DECEL_MM_S2            1500.0f

/* He so K_s cua luat vi tri: v = K_s * e (vung cuoi) */
#define AGV_POS_GAIN_KS                8.0f

/* Toc do toi thieu khi tiep can dich (thang ma sat) */
#define AGV_STOP_FLOOR_MM_S            30.0f

/* Sai so vi tri coi la da den dich */
#define AGV_STOP_TOL_MM                1.0f

/* Toc do khong duoc thap hon khi dang bam line (de bai: >= 0.1 m/s) */
#define AGV_MIN_TRACK_SPEED_MM_S       100.0f

/* Giam toc khi lech line lon */
#define AGV_CURVE_SLOW_GAIN            0.35f
#define AGV_CURVE_SLOW_REF_MM          25.0f

/* Mat line qua lau (ngoai vung cuoi) thi bao loi */
#define AGV_LOST_TIMEOUT_S             0.40f

/* ================= BO DIEU KHIEN BAM LINE (VONG NGOAI, LYAPUNOV) =================
 *
 * LINE_LAW_KANAYAMA : luat (5.13) cua bao cao
 *       v = v_R * cos(e3) + k1*e1          (e1 = 0 vi cam bien khong do doc line)
 *       w = k2 * e2 * v_R + w_R + k3 * sin(e3)
 *       V = (e1^2 + e2^2)/2 + (1 - cos e3)/k2
 *   e2 do tu day cam bien, e3 uoc luong boi bo quan sat (thay cho (5.17)).
 *   w_R = v_R * do_cong_line: tren xe that khong biet truoc nen = 0.
 *
 * LINE_LAW_PID_LYAP : ban PID + thich nghi cua minh (D tu quan sat, I thich nghi)
 */

#define LINE_LAW_KANAYAMA              0
#define LINE_LAW_PID_LYAP              1
#define LINE_LAW                       LINE_LAW_KANAYAMA

/* Kanayama:  k2 [rad / mm^2],  k3 [rad / s]   (k2, k3 > 0) */
#define LINE_K2                        2.0e-4f
#define LINE_K3                        1.0f

/* PID_LYAP:  K [1/s], C (0..1), GAMMA [rad/s tren mm*s] */
#define LYAP_K                         8.0f
#define LYAP_C                         0.8f
#define LYAP_GAMMA                     0.010f
#define LYAP_I_LIMIT                   0.8f        /* rad/s */

/* Bo quan sat (y, theta): tan so rieng va he so tat dan */
#define OBS_WN                         19.0f
#define OBS_ZETA                       1.0f

/* Van toc nho nhat dung trong bo quan sat (tranh chia cho 0) */
#define OBS_V_MIN_MM_S                 60.0f

/* Mat line: giu omega cu trong thoi gian nay roi ve 0 */
#define LINE_LOST_HOLD_S               0.15f

/* Gioi han toc do quay (rad/s) */
#define LINE_OMEGA_MAX                 2.5f

/* ================= NHAN BIET HANG VA MAU ================= */

/* Co hang khi kenh Clear THAY DOI so voi baseline (tang HOAC giam):
      |Clear - baseline| > baseline * PCT / 100 + DELTA
   (baseline do luc chua co hang, xe dang chay toi khu tai hang) */
#define LOAD_PRESENT_PCT               40U
#define LOAD_PRESENT_DELTA             30U

/* Hang phai on dinh lien tuc thoi gian nay moi xac nhan */
#define LOAD_CONFIRM_MS                800U

/* Chu ky doc cam bien (TCS34725 tich phan 50 ms) */
#define LOAD_SAMPLE_PERIOD_MS          60U

/* Phan biet mau: kenh troi hon kenh kia it nhat 1.3 lan */
#define LOAD_COLOR_DOMINANCE_X10       12U

/* ================= NHIEM VU ================= */

/* Quang duong tu luc xuat phat den diem dung tai hang (sa bo: 2000 mm) */
#define AGV_LOAD_ZONE_DIST_MM          2100.0f

/* Quang duong tu diem tai hang den cuoi line (ly thuyet = 4709 mm,
   sau do hieu chinh theo so do thuc te) */
#define AGV_END_PATH_LENGTH_MM         4610.0f

/* Vung cham truoc cuoi line */
#define AGV_END_SLOW_DIST_MM           100.0f
#define AGV_END_SLOW_SPEED_MM_S        100.0f

/* Neu qua dich them quang nay ma van chua thay cuoi line thi dung */
#define AGV_END_OVERRUN_MM             200.0f

/* Cho phep nhan biet cuoi line som hon dich nay (sai so odometry) */
#define AGV_END_DETECT_MARGIN_MM       200.0f

/* So mau lien tiep mat line moi xem la het line */
#define AGV_END_CONFIRM_SAMPLES        1U

/* Quang duong chay them sau khi phat hien het line */
#define AGV_END_EXTRA_MM               200.0f

/* Nhanh tren la DO (re trai), nhanh duoi la XANH (re phai).
   Neu sa ban nguoc lai thi doi 2 dong nay. */
#define AGV_BRANCH_RED                 LINE_PREF_LEFT
#define AGV_BRANCH_BLUE                LINE_PREF_RIGHT

#define AGV_START_DELAY_MS             3000U

/* Thoi gian cho nguoi rut tay ra sau khi nhan duoc hang */
#define AGV_LOAD_SETTLE_MS             1500U

#endif /* ROBOT_CONFIG_H */
