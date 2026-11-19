
#ifndef MOTION_PROFILE_H
#define MOTION_PROFILE_H

#include <stdint.h>

/* Bat dau quy dao: dung tai quang duong target_mm (tinh tu luc bat dau segment) */
void MotionProfile_Start(float target_mm, float cruise_mm_s);

void MotionProfile_SetTarget(float target_mm);
void MotionProfile_SetCruise(float cruise_mm_s);
void MotionProfile_SetDecel(float decel_mm_s2);

/* Tra ve van toc tien mong muon (mm/s). s = quang duong hien tai (mm) */
float MotionProfile_Update(float s, float dt);

/* 1 khi da den dich (sai so <= AGV_STOP_TOL_MM) */
uint8_t MotionProfile_IsDone(void);

float MotionProfile_GetTarget(void);

#endif /* MOTION_PROFILE_H */
