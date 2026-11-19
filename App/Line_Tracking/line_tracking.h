
#ifndef LINE_TRACKING_H
#define LINE_TRACKING_H

#include "main.h"
#include "line_sensor.h"
#include "robot_config.h"

/* Dat lai bo quan sat / tich phan (goi khi bat dau 1 segment) */
void LineTracking_Reset(void);

/* Tinh toc do quay mong muon (rad/s), DUONG = re PHAI.
 * Phai goi LineSensor_Update() truoc.
 *   v_mm_s     : van toc tien do duoc (mm/s)
 *   omega_meas : toc do quay do duoc tu encoder (rad/s, duong = phai) */
float LineTracking_Compute(float dt, float v_mm_s, float omega_meas);

uint8_t LineTracking_IsLost(void);

/* Thoi gian mat line lien tuc (s) */
float LineTracking_GetLostTime(void);

/* Vi tri line so voi tam hang mat (mm), DUONG = line ben PHAI */
float LineTracking_GetError(void);

/* Uoc luong goc lech (rad) cua bo quan sat */
float LineTracking_GetHeadingEst(void);

#endif /* LINE_TRACKING_H */
