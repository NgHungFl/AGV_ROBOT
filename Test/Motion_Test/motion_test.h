
#ifndef MOTION_TEST_H
#define MOTION_TEST_H

typedef enum
{
    MOTION_TEST_DUTY = 0,       /* do quan he duty - toc do tung dong co (NHAC BANH KHOI SAN) */
    MOTION_TEST_SPEED,      /* dap ung nac vong toc do banh (chinh KP, KI, KFF) */
    MOTION_TEST_STRAIGHT,       /* chay thang 1000 mm khong bam line (hieu chuan mm/xung) */
    MOTION_TEST_FOLLOW          /* bam line 3000 mm (chinh luat Lyapunov / PID) */
} MotionTest_t;

void MotionTest_Init(MotionTest_t test);

void MotionTest_Run(void);

#endif
