
#ifndef LINE_TRACKING_H
#define LINE_TRACKING_H

#include "main.h"
#include "motor.h"

/* ================= CAU HINH BAM LINE ================= */

/* Toc do co ban cua xe: 0 - 100 */
#define LINE_BASE_SPEED              25

/* Thong so PID - gia tri khoi dau de thu nghiem */
#define LINE_KP                      0.010f
#define LINE_KI                      0.0001f
#define LINE_KD                      0.00005f

/* Gioi han muc dieu chinh toc do */
#define LINE_MAX_CORRECTION          20

/* Gioi han tich phan */
#define LINE_INTEGRAL_LIMIT          5000.0f

/* Chu ky dieu khien: 10 ms */
#define LINE_CONTROL_PERIOD          10U

/* ================= NHAN DIEN VACH NGANG ================= */

/* 1: Bat tinh nang dung khi ca 5 mat deu thay den
   0: Tat tinh nang nay */
#define LINE_ALL_BLACK_STOP_ENABLE   1

/* Nguong nhan dien mau den: gia tri 0 - 1000 */
#define LINE_ALL_BLACK_THRESHOLD     700

/* Ca 5 mat deu den lien tuc trong 50 ms thi dung */
#define LINE_ALL_BLACK_TIME          50U

/* ================= CAC HAM ================= */

Motor_Status_t LineTracking_Init(void);

void LineTracking_Run(void);

void LineTracking_Stop(void);

#endif
