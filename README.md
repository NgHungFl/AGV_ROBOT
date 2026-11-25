# AGV_ROBOT

Tai lieu nay mo ta luong chay, cac module, mo hinh dieu khien, thong so hieu chinh va cac che do test cua firmware AGV.

> **Pham vi:** tai lieu duoc viet theo source hien co trong repository. Gia tri trong `robot_config.h` va `line_sensor.h` la cau hinh dang co; nhieu thong so co ghi chu can do lai tren robot that.

## 1. Tong quan

Robot la xe AGV hai banh vi sai, dung STM32F103xB va STM32 HAL. Firmware dang co cac chuc nang:

- Doc 5 mat line bang ADC1 scan + DMA.
- Doc encoder rieng cho banh trai va banh phai.
- Bam line bang phan hoi trang thai theo Lyapunov va bo quan sat Luenberger.
- Dieu khien toc do tung banh bang feed-forward theo model dong co ket hop PI.
- Doc cam bien mau TCS34725, phat hien co hang va phan loai do/xanh.
- Chay cac doan duong, giam toc, re nhanh theo mau va dung tai cuoi line.
- In telemetry va ket qua test qua UART.

Che do mac dinh trong `Core/Src/main.c` la `APP_MODE_MISSION`.

## 2. Cau truc thu muc

| Thu muc | Vai tro |
|---|---|
| `Core/` | Khoi tao MCU, clock, GPIO, ADC, DMA, I2C, timer, UART va interrupt. |
| `App/AGV_Mission/` | May trang thai cua nhiem vu: khoi hanh, cho hang, doc mau, chon nhanh, ket thuc. |
| `App/AGV_Motion/` | Vong dieu phoi chuyen dong; ket hop odometry, motion profile, bam line va toc do hai banh. |
| `App/Line_Tracking/` | Observer va bo dieu khien bam line. |
| `App/Load_Detect/` | Tao baseline Clear, phat hien vat the va phan loai mau trong nhiem vu. |
| `App/Motion_Profile/` | Tao van toc tham chieu va quy dao dung theo quang duong. |
| `App/Wheel_Control/` | Vong dieu khien toc do tung banh. |
| `App/Motor_Model/` | Feed-forward duty-toc do rieng theo banh va chieu quay. |
| `App/PID/` | Bo PID dung chung cho cac module. |
| `App/Robot_Config/` | Thong so co khi, encoder, dong co, PID, line, mau va nhiem vu. |
| `BSP/` | Driver muc board cho motor, encoder, line sensor, TCS34725, UART va timer. |
| `Test/` | Cac chuong trinh kiem tra encoder, motor, line, mau va motion. |
| `Tools/Python/` | Script monitor UART va mo phong C. |
| `Drivers/`, `cmake/` | STM32 CMSIS/HAL va cau hinh build CMake/CubeMX. |

File nen doc dau tien:

1. `Core/Src/main.c`: chon mode, khoi tao va vong lap chinh.
2. `App/Robot_Config/robot_config.h`: tham so dieu chinh.
3. `App/AGV_Mission/agv_mission.c`: trinh tu nhiem vu.
4. `App/AGV_Motion/agv_motion.c`: luong dieu khien chuyen dong moi chu ky.
5. Cac driver va bo dieu khien duoc lien ket trong bang cau truc thu muc.

## 3. Luong khoi dong va chu ky 10 ms

Trong che do nhiem vu, `main()` khoi tao clock/peripheral, UART, line sensor, motor, encoder, TCS34725, motion control va timer TIM2. Sau do:

- Vong `while (1)` goi `AGV_Mission_Run()` de xu ly state machine va in telemetry.
- Ngat TIM2 goi `AGV_Motion_Tick()` theo chu ky 10 ms de cap nhat encoder, line va cac vong dieu khien.
- ADC line chay scan lien tuc va ghi ket qua vao buffer DMA. `LineSensor_Update()` xu ly snapshot buffer moi trong tick dieu khien; do do 10 ms la chu ky dieu khien/doc-xu-ly, khong phai trigger ADC dung moi 10 ms.
- TCS34725 duoc doc trong luong nhiem vu foreground, khong doc trong ngat dieu khien. Chu ky doc duoc gioi han boi `LOAD_SAMPLE_PERIOD_MS`.

### Tinh chu ky TIM2

Cau hinh clock dat SYSCLK = 72 MHz, APB1 chia 2. Clock timer APB1 duoc nhan doi khi APB1 prescaler khac 1, nen TIM2 nhan 72 MHz. Trong `BSP/Control_Timer/control_timer.c`:

$$
f_{\mathrm{tick}} = \frac{72\,000\,000}{7199+1}
                 = 10\,000\ \mathrm{Hz}
$$

Moi tick timer bang 0.1 ms. Khi `ROBOT_CONTROL_PERIOD_MS = 10`, auto-reload register la `10 * 10 - 1 = 99`:

$$
T_{\mathrm{update}} =
\frac{99+1}{10\,000}
= 0.010\ \mathrm{s}
= 10\ \mathrm{ms}
$$

`Core/Src/main.c` truyen `AGV_Motion_Tick` vao `ControlTimer_Init()`. `TIM2_IRQHandler()` chuyen su kien qua HAL, sau do `HAL_TIM_PeriodElapsedCallback()` goi callback da dang ky.

`ROBOT_CONTROL_DT_S` cung duoc dat la `0.010f`. Gia tri nay duoc truyen vao cac cap nhat encoder, motion profile va PID; neu thay chu ky timer thi can dam bao `dt` dong bo.

## 4. Luong nhiem vu day du

May trang thai trong `App/AGV_Mission/agv_mission.c`:

1. **`MISSION_WAIT_START`**: cho `AGV_START_DELAY_MS`. Neu TCS34725 khong khoi tao duoc thi chuyen sang fault.
2. **`MISSION_GO_LOAD`**: xe bam line den `AGV_LOAD_ZONE_DIST_MM`. Trong luc di, `LoadDetect_UpdateBaseline()` theo doi muc sang nen.
3. **`MISSION_WAIT_LOAD`**: xe da dung tai vung hang; cap nhat mau, xac nhan vat the va phan loai do/xanh.
4. **`MISSION_LOAD_SETTLE`**: cho nguoi rut tay/cho hang on dinh trong `AGV_LOAD_SETTLE_MS`.
5. **`MISSION_GO_END`**: chon `AGV_BRANCH_RED` hoac `AGV_BRANCH_BLUE`, bam nhanh tuong ung va chay den cuoi line.
6. **`MISSION_DONE`**: hoan tat.
7. **`MISSION_FAULT`**: loi cam bien mau, mat line qua lau hoac khong tim thay cuoi line theo quy dinh.

Hai doan duong mac dinh:

- Den khu vuc hang: `target_mm = AGV_LOAD_ZONE_DIST_MM`, bam line, dung theo odometry.
- Tu khu hang den cuoi: `target_mm = AGV_END_PATH_LENGTH_MM`, bat phat hien cuoi line, giam toc gan cuoi va co gioi han overrun.

Mot tick motion trong `App/AGV_Motion/agv_motion.c` co thu tu:

1. Cap nhat encoder.
2. Cap nhat du lieu 5 mat line.
3. Cap nhat quang duong va van toc profile.
4. Tinh `omega_cmd` tu bo bam line, neu doan duong yeu cau bam line.
5. Giam toc theo do lech line, gioi han quay va tinh hai toc do banh.
6. Cap nhat duty motor qua wheel controller.

Khi motion khong chay, `WheelControl_Hold()` yeu cau phanh dong nang. Khi ket thuc doan, trang thai motion duoc phan biet la da den target, da phat hien line end hay fault.

## 5. Cam bien va odometry

### 5.1. Nam mat line

ADC1 scan 5 kenh PA0..PA4, DMA luu ket qua. Thu tu vat ly trong code la S1..S5 = PA4, PA3, PA2, PA1, PA0. Moi cam bien duoc chuan hoa bang gia tri BLACK va WHITE rieng:

$$
q_i =
\operatorname{clamp}
\left(
1000\frac{raw_i-white_i}{black_i-white_i},
0,\ 1000
\right)
$$

Trong do `q_i = 0` gan voi nen trang va `q_i = 1000` gan voi vach den. Neu `black_i == white_i`, code tra ve 0 de tranh chia cho 0.

Cam bien duoc xem la thay line khi `q_i >= LINE_DETECT_THRESHOLD`. Vi tri line la trung binh co trong so cua cac mat dang active:

$$
p =
\frac{\sum_{i \in A} w_i q_i}
     {\sum_{i \in A} q_i}
$$

`w_i` hien dat tu -2000 den +2000 tu trai sang phai. Chuyen sang millimet:

$$
d_{\mathrm{line}} =
p\frac{\mathrm{ROBOT\_SENSOR\_PITCH\_MM}}{1000}
$$

Quy uoc trong source: vi tri duong la line ben phai. Neu tong gia tri active bang 0, cam bien bao `LINE_LOST`.

Khi chua chon nhanh (`LINE_PREF_NONE`), code tinh vi tri tren tat ca mat active. Khi chon nhanh trai/phai, code chon blob active ben trai nhat/ben phai nhat. Neu vet active rong hon ba cam bien, code chi lay ba cam bien o phia nhanh da chon.

### 5.2. Encoder va quang duong

TIM1 doc encoder banh trai, TIM4 doc encoder banh phai. Code tinh so count chenh lech giua hai tick, nhan `ENCODER_LEFT_SIGN`/`ENCODER_RIGHT_SIGN`, tich luy count va doi sang quang duong:

$$
N_{\mathrm{counts/rev}} =
PPR \times MULT \times GEAR
$$

$$
mm_{\mathrm{per\ count}} =
\frac{\mathrm{ROBOT\_WHEEL\_CIRCUMFERENCE\_MM}}
     {N_{\mathrm{counts/rev}}}
$$

$$
s_i = count_i \times mm_{\mathrm{per\ count}}
$$

Toc do raw moi tick la:

$$
v_{i,\mathrm{raw}} =
\frac{\Delta count_i \times mm_{\mathrm{per\ count}}}{dt}
$$

Toc do duoc loc EMA:

$$
v_i \leftarrow v_i + \alpha(v_{i,\mathrm{raw}}-v_i)
$$

Quang duong va toc do tam xe la trung binh hai banh:

$$
s = \frac{s_L+s_R}{2},
\qquad
v = \frac{v_L+v_R}{2}
$$

Toc do quay duong la re phai:

$$
\omega_{\mathrm{meas}} =
\frac{v_L-v_R}{W}
$$

`W` la `ROBOT_TRACK_WIDTH_MM`. Voi cau hinh hien tai, 11 PPR x4 x ti so truyen 30 = 1320 count/vong va chu vi 200 mm thi code tinh xap xi 0.1515 mm/count. Hay do chu vi banh va xac minh chieu encoder tren robot that.

## 6. Bám line theo mô hình Chương 5

Chế độ mặc định `LINE_LAW_STATE_FEEDBACK` dùng luật (5.27). `M` là trung điểm trục bánh, `C` là tâm dãy cảm biến và `l` là khoảng cách từ `M` tới `C`. Firmware quy ước chiều dương về **phải** cho sai số ngang, sai số góc và tốc độ quay. Đổi dấu cả ba đại lượng khi đối chiếu với quy ước dương về trái của Chương 5.

Dãy 5 mắt trả về vị trí vạch `d` so với tâm `C`. Ở đoạn thẳng với góc nhỏ, sai số tại `C` xấp xỉ `−d`. Bộ quan sát lưu `eC_hat` và `theta_hat`, rồi tính `e2_hat = eC_hat − l*theta_hat` để phản hồi tại `M`. Lệnh quay theo (5.27) là $\omega_{\rm cmd}=-k_2\hat e_2-k_3\sin(\hat\theta)$.

### Bộ quan sát

Encoder cung cấp `v_M=(vL+vR)/2` và tốc độ quay phải `omega_meas=(vL−vR)/W`. Với sai lệch đo trừ dự đoán `epsilon = −d − eC_hat` sau khi kẹp, firmware tích phân Euler với bước `dt`:

$$
\begin{aligned}
\hat e_{C,k+1} &= \hat e_{C,k} + dt\,[v_M\hat\theta_k+l\,\omega_{\rm meas}+(2\zeta_o\omega_o)\epsilon_k],\\
\hat\theta_{k+1} &= \hat\theta_k + dt\,[\omega_{\rm meas}+L_2\epsilon_k].
\end{aligned}
$$

Gain hiệu chỉnh của `eC_hat` bằng `L1 + l*L2` theo (5.18). Khi `v_M` từ `OBS_V_MIN_MM_S` trở lên, `L2 = ωo²/v_M`. Dưới ngưỡng này, firmware dùng `L2 = ωo²*v_M/OBS_V_MIN_MM_S²` cho chuyển động tiến để hiệu chỉnh góc giảm dần về 0 khi xe dừng. Khi mất vạch, bộ quan sát chỉ dự đoán từ encoder và giữ lệnh quay tối đa `LINE_LOST_HOLD_S`. Sau đó lệnh quay về 0 và bộ quan sát khởi tạo lại ở lần thấy vạch tiếp theo.

### Chọn hệ số

- `LINE_STATE_K2` tác động lên sai số ngang tại `M`. `LINE_STATE_K3` tác động lên góc lệch. Theo (5.26), ở vận tốc thiết kế $v_0$, có thể lấy $k_2=\omega_c^2/v_0$ và $k_3=2\zeta_c\omega_c$ để chọn bộ gain thử ban đầu.
- Với $v_0=500$ mm/s và bộ gain hiện tại, mô hình tuyến tính lý tưởng cho $\omega_c\approx8.94$ rad/s và $\zeta_c\approx0.335$. `OBS_STATE_WN` và `OBS_ZETA` tương ứng với $\omega_o=16$ rad/s và $\zeta_o=0.9$. Tại tốc độ này, $L_2=0.512$ rad/(mm·s) và $L_1\approx5.50$ s⁻¹.
- `AGV_Motion` giới hạn thêm lệnh quay theo $|\omega|\leq 2v_{\rm cmd}/W$ để bánh trong không có tốc độ đặt âm. Vận tốc tiến đến từ motion profile và hệ số giảm tốc theo độ lệch vạch.
- Chứng minh Lyapunov trong Chương 5 xét đoạn thẳng, tốc độ tiến dương không đổi và phản hồi trạng thái lý tưởng. Trên cua, góc tiếp tuyến thay đổi và luật hiện hành không có bù độ cong của vạch. Đánh giá đáp ứng trên cua bằng mô phỏng và thử xe.

Hai chế độ `LINE_LAW_KANAYAMA` và `LINE_LAW_PID_LYAP` có bộ gain riêng. Khi thử cấu hình cũ, đổi `LINE_LAW` để dùng đúng quy ước và đơn vị của các gain cũ.

### Chạy mô phỏng và kiểm chứng

Dùng Python 3 và GCC cho máy tính. Ví dụ PowerShell với GCC (MSYS2 UCRT64) trong `PATH`:

```powershell
python Tools/Python/Sim/build.py build/sim/chapter5.exe
python Tools/Python/Sim/build.py build/sim/kanayama.exe '-DLINE_LAW=LINE_LAW_KANAYAMA'
.\build\sim\chapter5.exe blue offset=-15 heading=-8 csv=build/sim/chapter5.csv
.\build\sim\kanayama.exe blue offset=-15 heading=-8
```

Để thử giá trị khác, truyền `-DLINE_STATE_K2=…`, `-DLINE_STATE_K3=…`, `-DOBS_STATE_WN=…` hoặc `-DAGV_CRUISE_SPEED_MM_S=…` khi build mô phỏng. `offset` là độ lệch ban đầu của `C` tính bằng mm, `heading` tính bằng độ, cả hai dương về phải. `L` và `W` thay đổi kích thước của xe mô phỏng để thử sai lệch so với cấu hình firmware. `blue` chọn nhánh xanh, mặc định là nhánh đỏ.

Bảng ghi trung bình RMS của 7 tình huống ở mỗi tốc độ hành trình, với gia tốc và giảm tốc 500 mm/s². Các tình huống gồm hai nhánh, lệch ban đầu, sai lệch kích thước và sai khác động cơ. Sai số tính theo khoảng cách từ tâm dãy mắt đến tim vạch trên đường dự kiến khi còn bám line.

| Tốc độ hành trình đặt (mm/s) | RMS luật cũ (mm) | RMS luật (5.27) (mm) |
|---:|---:|---:|
| 250 | 5.11 | 2.48 |
| 325 | 6.02 | 2.83 |
| 400 | 6.70 | 3.40 |
| 500 | 7.24 | 4.68 |

Bộ gain trên hoàn thành 28/28 lượt và không mất vạch trong khoảng bám được tính. Mô phỏng dùng mô hình động cơ bậc nhất, encoder và dãy cảm biến giả lập. Kết quả dùng để chọn bộ gain khởi đầu cho xe thật.

## 7. Quy dao van toc va dong hoc hai banh

### 7.1. Motion profile dung theo quang duong

Trong `App/Motion_Profile/motion_profile.c`, sai so con lai:

$$
e = s_{\mathrm{target}}-s
$$

Quy dao phanh dua tren:

$$
v_{\mathrm{brake}}=\sqrt{2ae}
$$

Gan dich, code them luat tuyen tinh de giam do doc:

$$
v_{\mathrm{linear}}=K_s e
$$

Van toc mong muon la gia tri nho nhat giua hai luat tren va toc do cruise, sau do ap dung toc do floor va gioi han gia toc:

$$
v_{\mathrm{des}} =
\min\left(\sqrt{2ae},K_s e,v_{\mathrm{cruise}}\right)
$$

Doan code gioi han thay doi van toc moi tick:

$$
\Delta v_{\mathrm{up}}\le a_{\mathrm{accel}}dt
$$

$$
\Delta v_{\mathrm{down}}\ge -1.5\,a_{\mathrm{decel}}dt
$$

Khi `e <= AGV_STOP_TOL_MM`, profile bao da den dich va tra van toc 0.

### 7.2. Giam toc khi lech line va inverse kinematics

`AGV_Motion_Run()` giam toc khi do lech line lon:

$$
k_{\mathrm{slow}} =
1-g\cdot
\min\left(\frac{|d_{\mathrm{line}}|}{d_{\mathrm{ref}}},1\right)
$$

$$
v_{\mathrm{cmd}}=v_{\mathrm{ref}}k_{\mathrm{slow}}
$$

Neu dang chay nhanh hon toc do toi thieu bam line, code giu `v_cmd` khong thap hon `AGV_MIN_TRACK_SPEED_MM_S`.

Tu toc do tien va toc do quay, toc do tham chieu hai banh:

$$
v_L=v_{\mathrm{cmd}}+\frac{\omega_{\mathrm{cmd}}W}{2}
$$

$$
v_R=v_{\mathrm{cmd}}-\frac{\omega_{\mathrm{cmd}}W}{2}
$$

De banh trong khong can dao chieu khi xe dang re, code gioi han:

$$
|\omega_{\mathrm{cmd}}|
\le \frac{2v_{\mathrm{cmd}}}{W}
$$

## 8. Vong dieu khien toc do va model dong co

### 8.1. Model feed-forward

Trong `App/Motor_Model/motor_model.c`, moi banh va moi chieu co gain/deadband rieng. Vi du voi chieu tien:

$$
v = G_{\mathrm{fwd}}(u-DB_{\mathrm{fwd}})
\quad \text{khi }u>DB_{\mathrm{fwd}}
$$

Chieu lui:

$$
v = G_{\mathrm{rev}}(u+DB_{\mathrm{rev}})
\quad \text{khi }u<-DB_{\mathrm{rev}}
$$

Trong do `u` la duty co dau theo phan tram, `v` la mm/s. Ham nguoc tao duty feed-forward:

$$
u_{\mathrm{ff}} =
\begin{cases}
\frac{v_{\mathrm{ref}}}{G_{\mathrm{fwd}}}+DB_{\mathrm{fwd}},
&v_{\mathrm{ref}}>1\\
\frac{v_{\mathrm{ref}}}{G_{\mathrm{rev}}}-DB_{\mathrm{rev}},
&v_{\mathrm{ref}}<-1\\
0,&|v_{\mathrm{ref}}|\le1
\end{cases}
$$

### 8.2. PI toc do

Moi banh co gain PI rieng theo rpm. Code chia gain theo `ROBOT_MM_S_PER_RPM` de dung sai so mm/s. Hai banh dung cung `WHEEL_INTEGRAL_LIMIT`.

$$
e_i=v_{i,\mathrm{ref}}-v_{i,\mathrm{encoder}}
$$

$$
u_{\mathrm{trim},i}
=K_p e_i+I_i,
\qquad
I_i\leftarrow I_i+K_i e_i dt
$$

Duty cuoi ket hop feed-forward va trim:

$$
u_i=\operatorname{clamp}
\left(u_{\mathrm{ff},i}+u_{\mathrm{trim},i},-100,100\right)
$$

Code dieu chinh gioi han PID trim theo `u_ff` de tong duty khong vuot +/-100%. PID co anti-windup va gioi han tich phan.

### 8.3. Xuat duty va phanh

`BSP/Motor/motor.c` dieu khien TB6612FNG bang:

- TIM3 CH1/CH2 cho PWM hai motor.
- GPIO chon chieu quay.
- Duty phan tram duoc doi sang khoang PWM 0..3599.
- `Motor_BrakeAll()` dung phanh dong nang; `Motor_StopAll()` tha motor (coast); `Motor_Disable()` tha motor va dua driver vao standby.

## 9. Doc va phan loai hang

`BSP/Color_Sensor/tcs34725.c` khoi tao TCS34725 qua I2C1, kiem tra ID, dat thoi gian tich phan khoang 50.4 ms va gain 4x. Moi lan doc, code lay 4 kenh Clear, Red, Green, Blue.

Trong luc xe chua den hang, `LoadDetect_UpdateBaseline()` cap nhat baseline Clear bang EMA:

$$
b_{k+1}=b_k+\frac{x_k-b_k}{8}
$$

Khi cho hang, code tinh:

$$
T = \frac{b \times \mathrm{LOAD\_PRESENT\_PCT}}{100}
+ \mathrm{LOAD\_PRESENT\_DELTA}
$$

$$
\mathrm{present}
\iff |Clear-b|>T
$$

Sau khi phat hien hang, code doi `LOAD_CONFIRM_MS/2` dau truoc khi cong mau RGB vao tong; sau `LOAD_CONFIRM_MS`, phan loai tren tong mau:

$$
Red \iff 10\sum R > D\sum B
$$

$$
Blue \iff 10\sum B > D\sum R
$$

Trong code `D = LOAD_COLOR_DOMINANCE_X10`; neu khong dat ben nao thi mau chua phan biet duoc va lay lai mau. Kenh G duoc doc va cong nhung khong dung trong quy tac do/xanh cua `LoadDetect`.

**Luu y khac biet:** comment trong `robot_config.h` ghi nguong mau 1.3 lan, nhung `LOAD_COLOR_DOMINANCE_X10 = 12` va cong thuc thuc thi tuong ung 1.2 lan. Ngoai ra `TCS34725_ClassifyColor()` trong driver la helper khac; nhiem vu chinh dung bo phan loai rieng trong `LoadDetect_Classify()`.

## 10. Cac thong so cau hinh

Thong so chinh nam trong `App/Robot_Config/robot_config.h`; calibration 5 mat line nam trong `BSP/Line_Sensor/line_sensor.h`.

### Chu ky va co khi

| Macro | Y nghia | Gia tri hien tai |
|---|---|---:|
| `ROBOT_CONTROL_PERIOD_MS` | Chu ky TIM2 | 10 ms |
| `ROBOT_CONTROL_DT_S` | `dt` truyen vao controller | 0.010 s |
| `ROBOT_WHEEL_CIRCUMFERENCE_MM` | Chu vi banh trong odometry | 200 mm |
| `ROBOT_TRACK_WIDTH_MM` | Khoang cach tam hai banh | 200 mm, can do |
| `ROBOT_SENSOR_DIST_MM` | Khoang cach truc banh den hang mat | 45.5 mm |
| `ROBOT_SENSOR_PITCH_MM` | Khoang cach giua hai mat line | 15 mm |

### Encoder va motor

| Macro | Y nghia | Gia tri hien tai |
|---|---|---:|
| `ROBOT_ENC_PPR` | Xung encoder moi vong truc motor | 11 |
| `ROBOT_ENC_MULT` | He so dem quadrature | 4 |
| `ROBOT_GEAR_RATIO` | Ti so truyen | 30 |
| `ENCODER_LEFT_SIGN`, `ENCODER_RIGHT_SIGN` | Chuan hoa chieu count | `+1`, `-1` |
| `ENCODER_SPEED_ALPHA` | He so loc toc do EMA | 0.4 |
| `MOTOR_L/R_GAIN_FWD/REV` | Gain model moi banh/chieu | Rieng tung banh |
| `MOTOR_L/R_DB_FWD/REV` | Deadband model moi banh/chieu | Rieng tung banh |
| `MOTOR_LEFT_REVERSE`, `MOTOR_RIGHT_REVERSE` | Dao chieu dau duty phan cung | 0 |
| `WHEEL_L_KP_RPM`, `WHEEL_L_KI_RPM` | He so PI banh trai, sai so theo rpm | 0.50643, 5.7877 |
| `WHEEL_R_KP_RPM`, `WHEEL_R_KI_RPM` | He so PI banh phai, sai so theo rpm | 0.50218, 5.2861 |
| `WHEEL_INTEGRAL_LIMIT` | Gioi han tich phan PI toc do | 40% |

### Van toc va bam line

| Macro | Y nghia | Gia tri hien tai |
|---|---|---:|
| `AGV_CRUISE_SPEED_MM_S` | Toc do cruise mac dinh | 500 mm/s |
| `AGV_ACCEL_MM_S2` | Gioi han tang toc | 500 mm/s^2 |
| `AGV_DECEL_MM_S2` | Giam toc thong thuong | 500 mm/s^2 |
| `AGV_END_DECEL_MM_S2` | Giam toc khi da latch cuoi line | 1500 mm/s^2 |
| `AGV_POS_GAIN_KS` | He so luat dung tuyen tinh gan dich | 8 |
| `AGV_STOP_FLOOR_MM_S` | Toc do floor motion profile | 30 mm/s |
| `AGV_STOP_TOL_MM` | Sai so vi tri xem la da den | 1 mm |
| `AGV_MIN_TRACK_SPEED_MM_S` | Toc do toi thieu khi bam line | 100 mm/s |
| `AGV_CURVE_SLOW_GAIN` | Muc giam toc theo do lech | 0.30 |
| `AGV_CURVE_SLOW_REF_MM` | Do lech de dat muc giam toc toi da | 35 mm |
| `LINE_STATE_K2`, `LINE_STATE_K3` | Gain ngang tai M va gain goc cua (5.27) | 0.16 rad/(s·mm), 6 rad/s |
| `OBS_STATE_WN`, `OBS_ZETA` | Tan so rieng va he so tat dan cua bo quan sat | 16 rad/s, 0.9 |
| `OBS_V_MIN_MM_S` | Moc toc do de giam hieu chinh goc | 150 mm/s |
| `OBS_INNOV_MAX_MM`, `OBS_THETA_MAX_RAD` | Gioi han sai lech do du doan va goc uoc luong | 12 mm, 0.8 rad |
| `LINE_LOST_HOLD_S` | Thoi gian giu omega khi mat line | 0.15 s |
| `LINE_OMEGA_MAX` | Gioi han toc do quay line controller | 10 rad/s |
| `AGV_LOST_TIMEOUT_S` | Timeout mat line ngoai vung cuoi | 0.40 s |

### Line sensor, mau va duong nhiem vu

| Macro/nhom | Y nghia |
|---|---|
| `LINE_S1..S5_BLACK/WHITE` | ADC calib tung mat tren vach den/nen trang. Gia tri hien tai chi la vi du. |
| `LINE_DETECT_THRESHOLD` | Nguong `q_i` de xem mat do thay vach. |
| `LINE_WEIGHT_S1..S5` | Toa do trong so tung mat, tu trai sang phai. |
| `LOAD_PRESENT_PCT`, `LOAD_PRESENT_DELTA` | Nguong thay doi Clear de phat hien hang. |
| `LOAD_CONFIRM_MS` | Thoi gian phai on dinh truoc khi chot hang/mau. |
| `LOAD_SAMPLE_PERIOD_MS` | Chu ky doc TCS34725 tu state machine. |
| `LOAD_COLOR_DOMINANCE_X10` | He so so sanh tong R va B. |
| `AGV_LOAD_ZONE_DIST_MM` | Quang duong toi khu vuc hang. |
| `AGV_END_PATH_LENGTH_MM` | Quang duong danh nghia den cuoi line. |
| `AGV_END_SLOW_DIST_MM`, `AGV_END_SLOW_SPEED_MM_S` | Vung va toc do cham truoc dich. |
| `AGV_END_OVERRUN_MM` | Gioi han chay qua target khi chua thay line end. |
| `AGV_END_DETECT_MARGIN_MM` | Cho phep phat hien cuoi line som hon target. |
| `AGV_END_CONFIRM_SAMPLES` | So lan cap nhat line lost de xac nhan end. |
| `AGV_END_EXTRA_MM` | Quang duong them sau khi latch line end. |
| `AGV_BRANCH_RED`, `AGV_BRANCH_BLUE` | Anh xa mau hang sang nhanh trai/phai. |
| `AGV_START_DELAY_MS` | Tre truoc khi khoi hanh. |
| `AGV_LOAD_SETTLE_MS` | Thoi gian cho sau khi doc xong hang. |

Khong nen thay `ROBOT_CONTROL_PERIOD_MS` ma bo quen `ROBOT_CONTROL_DT_S`; tuong tu, cac thong so co khi can phu hop voi kich thuoc robot that.

## 11. Che do test va hieu chuan

Chon mode bang macro `APP_MODE` trong `Core/Src/main.c`, build va nap firmware lai.

| Gia tri | Mode | Muc dich |
|---:|---|---|
| 0 | `APP_MODE_MISSION` | Nhiem vu day du. |
| 1 | `APP_MODE_ENCODER_TEST` | Dung motor, xoay tay tung banh, xem count/chieu/distance. |
| 2 | `APP_MODE_SPEED_TEST` | Gui cac muc ref 0, 150, 300, 100 mm/s va in toc do/duty hai banh. |
| 3 | `APP_MODE_STRAIGHT_TEST` | Chay thang 1000 mm, cruise 250 mm/s. |
| 4 | `APP_MODE_FOLLOW_TEST` | Bam line doan 3000 mm. |
| 5 | `APP_MODE_LINE_SENSOR_TEST` | In RAW, normalized, vi tri va line status. |
| 6 | `APP_MODE_COLOR_TEST` | In RGB/C va ket qua classifier helper cua TCS34725. |
| 7 | `APP_MODE_DUTY_TEST` | Do model duty-toc do cho trai/phai va tien/lui. |
| 8 | `APP_MODE_LINE_CALIB` | Đo nền trắng và vạch đen cho 5 mắt, lưu hiệu chuẩn vào flash. |

Trinh tu hieu chuan de de khoanh vung loi:

1. Dùng mode hiệu chuẩn (`8`) trên đúng nền và vạch. Dùng mode cảm biến (`5`) để kiểm tra thứ tự S1–S5 và `POS` dương khi vạch nằm bên phải xe.
2. **Encoder (`1`)**: xoay tay tung banh dung chieu; count phai tang khi banh quay theo chieu tien da quy uoc. Xac minh encoder trai/phai khong bi doi.
3. **Model motor (`7`)**: chi test khi ke banh khoi san; chay het ca bon to hop motor/chieu, dung `GAIN/DB` in ra de cap nhat config.
4. **Vong toc do (`2`)**: so sanh `ref`, `vL`, `vR`, duty. Sau khi model feed-forward on dinh moi tinh chinh PI.
5. **Chay thang (`3`)**: kiem tra dau encoder, toc do hai banh va odometry.
6. Đo lại `ROBOT_TRACK_WIDTH_MM` và `ROBOT_SENSOR_DIST_MM`. Đặt `AGV_CRUISE_SPEED_MM_S` ở 250 mm/s cho bản test mode (`4`). Chạy trên đoạn thẳng với độ lệch nhẹ về hai phía, rồi qua cua và tăng dần đến tốc độ nhiệm vụ 500 mm/s khi xe bám ổn định. Dùng cùng vạch và điều kiện khởi đầu khi so sánh hai bộ gain. Nếu `w100` đổi dấu nhanh theo `th1000` trong khi `d10` ít đổi, thử giảm `OBS_STATE_WN`. Nếu hồi tâm chậm, tăng `LINE_STATE_K2` từng bước nhỏ và theo dõi dao động. Nếu sai số vượt qua tim vạch nhiều lần và `w100` chưa chạm giới hạn, thử tăng `LINE_STATE_K3`.
7. **Nhiem vu (`0`)**: cuoi cung moi chinh quang duong khu hang, nhanh mau va diem ket thuc.

`APP_MODE_MOTOR_TEST` chua duoc khai bao trong `main.c`; `Test/Motor_Test/motor_test.c` co ham test motor rieng nhung khong duoc goi boi cac nhanh mode hien tai.

### UART/log

UART3 duoc truu tuong hoa boi `BSP/UART/bsp_uart.c`; cac test va mission in ket qua bang `BSP_UART_Printf()`. Mot so log:

- Encoder test: `CNT_L`, `CNT_R`, `DIST_L`, `DIST_R`, `V_L`, `V_R`.
- Speed test: `t_ms,ref,vL,vR,duL_x10,duR_x10`.
- Follow test ở USART3, 115200 baud. `d10/10` là vị trí vạch so với C (mm), `eM10/10` là sai số ước lượng tại M (mm), `th1000/1000` là góc ước lượng (rad). Chia `w100` và `wmeas100` cho 100 để so sánh lệnh quay và tốc độ quay đo được (rad/s). `line=1` nghĩa là có vạch, `line=0` là mất vạch.
- Cho hang: Clear/R/G/B, baseline, threshold va `PRESENT`.

Float trong mot so log duoc nhan scale va ep sang so nguyen de phu hop cau hinh newlib-nano. Vi du `D10` can chia 10 de ra mm; `W100` can chia 100 de ra rad/s; `TH1000` can chia 1000 de ra rad.

`Tools/Python/line_sensor_monitor.py` doc output cua line sensor test qua serial; sua `COM_PORT` va baudrate cho dung may. `Tools/Python/color_sensor_monitor.py` la serial monitor don gian; test firmware hien in dong CSV.

## 12. Cac diem can xac minh tren phan cung

- Cac gia tri BLACK/WHITE trong `line_sensor.h` duoc ghi la vi du; can calib tren dung mat san va chieu cao cam bien.
- `ENCODER_TEST` comment trong source noi mot vong banh tuong ung khoang 100 mm, nhung cau hinh hien tai (`circumference = 200 mm`, 1320 count/vong) tinh ra 200 mm/vong. Hay do lai chu vi va cap nhat comment/cau hinh theo banh that.
- `LOAD_COLOR_DOMINANCE_X10 = 12` la nguong 1.2 lan trong cong thuc, trong khi comment ghi 1.3 lan.
- Classifier test mode 6 khac classifier nhiem vu: `TCS34725_ClassifyColor()` so ty le R tren tong RGB, trong khi `LoadDetect` so sanh R voi B theo nguong dominance.
- `MOTOR_LEFT_REVERSE`/`MOTOR_RIGHT_REVERSE` duoc ap dung trong API duty; API speed truc tiep (`Motor_SetSpeed`) khong su dung cac macro nay.
- Bao dam cap nhat dung encoder sign, motor direction va thuat toan nhan dang nhanh truoc khi tang toc do xe.
- Cac ket qua Lyapunov la dua tren mo hinh/gia thiet; saturate, chu ky roi rac, delay sensor, sai so co khi va ma sat co the lam response thuc te khac phan tich ly tuong.

## 13. Build

Du an dung CMake preset `Debug`/`Release`, Ninja va ARM GNU toolchain. Can cai dat CMake (toi thieu 3.22), Ninja va `arm-none-eabi-gcc` nam trong PATH.

```sh
cmake --preset Debug
cmake --build --preset Debug
```

Artifact firmware du kien: `build/Debug/AGV_ROBOT.elf`. CMake presets co them `Release`:

```sh
cmake --preset Release
cmake --build --preset Release
```

`Tools/Python/Sim/build.sh` la script mo phong C rieng, dung GCC host va stub HAL; no khong thay the build firmware ARM. Script nay su dung Bash va Python3.

## 14. Ghi chu ve cong thuc Markdown

Cong thuc trong tai lieu dung khoi display math `$$ ... $$` de cac renderer Markdown ho tro MathJax hien thi ro rang. Cac ten macro trong code duoc viet trong backtick de khong bi Markdown hieu nham.
