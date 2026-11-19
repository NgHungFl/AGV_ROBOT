
#ifndef AGV_MOTION_H
#define AGV_MOTION_H

#include "main.h"
#include "line_sensor.h"
#include <stdint.h>

typedef enum
{
    AGV_MOTION_IDLE = 0,
    AGV_MOTION_RUNNING,
    AGV_MOTION_ARRIVED,     /* den dich bang dem xung */
    AGV_MOTION_LINE_END,    /* dung do phat hien het line */
    AGV_MOTION_FAULT        /* mat line / khong thay cuoi line */
} AGV_MotionState_t;

typedef struct
{
    float   target_mm;          /* quang duong den dich (tu luc Start) */
    float   cruise_mm_s;

    uint8_t line_follow;        /* 1: bam line, 0: chay thang */

    /* Chi dung khi stop_on_line_end = 1 */
    uint8_t stop_on_line_end;
    float   slow_dist_mm;       /* cham truoc target mot doan */
    float   slow_speed_mm_s;
    float   overrun_mm;         /* qua target bao nhieu thi dung (loi) */
} AGV_Segment_t;

void AGV_Motion_Init(void);

/* Goi tu main. Bat dau 1 doan duong, quang duong tinh tu 0 */
void AGV_Motion_Start(const AGV_Segment_t *segment);

/* Goi trong ngat TIM2 moi ROBOT_CONTROL_PERIOD_MS */
void AGV_Motion_Tick(void);

AGV_MotionState_t AGV_Motion_GetState(void);

/* Du lieu cho telemetry */
float AGV_Motion_GetDistance(void);     /* mm, tinh tu luc Start */
float AGV_Motion_GetSpeedRef(void);     /* mm/s */
float AGV_Motion_GetOmegaCmd(void);     /* rad/s */
float AGV_Motion_GetEndDistance(void);  /* mm, quang duong luc phat hien het line */

void AGV_Motion_SetBranch(LinePreference_t pref);

#endif /* AGV_MOTION_H */
