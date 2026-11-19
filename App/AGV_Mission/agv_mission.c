
#include "agv_mission.h"
#include "robot_config.h"
#include "agv_motion.h"
#include "load_detect.h"
#include "encoder.h"
#include "line_tracking.h"
#include "bsp_uart.h"

#define MISSION_PRINT_PERIOD_MS   100U

static MissionState_t state = MISSION_WAIT_START;

static uint32_t state_tick  = 0;
static uint32_t print_tick  = 0;

static uint8_t color_ok = 0;

static LoadColor_t cargo = LOAD_COLOR_NONE;

/* ================= DOAN DUONG ================= */

static const AGV_Segment_t seg_to_load =
{
    .target_mm        = AGV_LOAD_ZONE_DIST_MM,
    .cruise_mm_s      = AGV_CRUISE_SPEED_MM_S,
    .line_follow      = 1,
    .stop_on_line_end = 0,
    .slow_dist_mm     = 0.0f,
    .slow_speed_mm_s  = 0.0f,
    .overrun_mm       = 0.0f
};

static const AGV_Segment_t seg_to_end =
{
    .target_mm        = AGV_END_PATH_LENGTH_MM,
    .cruise_mm_s      = AGV_CRUISE_SPEED_MM_S,
    .line_follow      = 1,
    .stop_on_line_end = 1,
    .slow_dist_mm     = AGV_END_SLOW_DIST_MM,
    .slow_speed_mm_s  = AGV_END_SLOW_SPEED_MM_S,
    .overrun_mm       = AGV_END_OVERRUN_MM
};

/* ================= NOI BO ================= */

static void Mission_SetState(MissionState_t next)
{
    state      = next;
    state_tick = HAL_GetTick();
}

/* In gia tri cam bien mau khi dang cho hang, de biet no doc duoc gi */
static void Mission_PrintLoad(void)
{
    static uint32_t load_tick = 0;
    const TCS34725_ColorData_t *c;

    if ((HAL_GetTick() - load_tick) < 300U)
    {
        return;
    }

    load_tick = HAL_GetTick();

    c = LoadDetect_GetSample();

    BSP_UART_Printf(
        "LOAD: C=%u R=%u G=%u B=%u | BASE=%u THR=%u PRESENT=%d\r\n",
        (unsigned)c->clear,
        (unsigned)c->red,
        (unsigned)c->green,
        (unsigned)c->blue,
        (unsigned)LoadDetect_GetBaseline(),
        (unsigned)LoadDetect_GetThreshold(),
        (int)LoadDetect_IsPresent());
}

static void Mission_Print(void)
{
    if (state == MISSION_WAIT_LOAD)
    {
        Mission_PrintLoad();
        return;
    }

    if ((HAL_GetTick() - print_tick) < MISSION_PRINT_PERIOD_MS)
    {
        return;
    }

    print_tick = HAL_GetTick();

    /* In so nguyen (x10) vi newlib-nano mac dinh khong in duoc float */
    BSP_UART_Printf(
        "M=%d,S=%ld,VREF=%ld,V=%ld,D10=%ld,W100=%ld,TH1000=%ld,ST=%d\r\n",
        (int)state,
        (long)AGV_Motion_GetDistance(),
        (long)AGV_Motion_GetSpeedRef(),
        (long)Encoder_GetCenterSpeed(),
        (long)(LineTracking_GetError() * 10.0f),
        (long)(AGV_Motion_GetOmegaCmd() * 100.0f),
        (long)(LineTracking_GetHeadingEst() * 1000.0f),
        (int)AGV_Motion_GetState());
}

/* ================= KHOI TAO ================= */

HAL_StatusTypeDef AGV_Mission_Init(I2C_HandleTypeDef *hi2c)
{
    HAL_StatusTypeDef status = LoadDetect_Init(hi2c);

    color_ok = (status == HAL_OK) ? 1U : 0U;

    AGV_Motion_SetBranch(LINE_PREF_NONE);

    Mission_SetState(MISSION_WAIT_START);

    return status;
}

MissionState_t AGV_Mission_GetState(void)
{
    return state;
}

/* ================= MAY TRANG THAI ================= */

void AGV_Mission_Run(void)
{
    AGV_MotionState_t motion = AGV_Motion_GetState();

    switch (state)
    {
        case MISSION_WAIT_START:

            if ((HAL_GetTick() - state_tick) >= AGV_START_DELAY_MS)
            {
                if (color_ok == 0U)
                {
                    BSP_UART_Printf("COLOR SENSOR ERROR\r\n");
                    Mission_SetState(MISSION_FAULT);
                    break;
                }

                BSP_UART_Printf("START\r\n");
                AGV_Motion_Start(&seg_to_load);
                Mission_SetState(MISSION_GO_LOAD);
            }

            break;

        case MISSION_GO_LOAD:

            /* Do muc sang nen luc chua co hang */
            LoadDetect_UpdateBaseline();

            if (motion == AGV_MOTION_ARRIVED)
            {
                BSP_UART_Printf("AT LOAD ZONE, S=%ld mm\r\n",
                                (long)AGV_Motion_GetDistance());

                LoadDetect_Reset();
                Mission_SetState(MISSION_WAIT_LOAD);
            }
            else if (motion == AGV_MOTION_FAULT)
            {
                BSP_UART_Printf("FAULT WHILE GO LOAD\r\n");
                Mission_SetState(MISSION_FAULT);
            }

            break;

        case MISSION_WAIT_LOAD:

            if (LoadDetect_Update() == LOAD_STATUS_READY)
            {
                cargo = LoadDetect_GetColor();

                BSP_UART_Printf("CARGO: %s\r\n",
                                (cargo == LOAD_COLOR_RED) ? "RED" : "BLUE");

                Mission_SetState(MISSION_LOAD_SETTLE);
            }

            break;

        case MISSION_LOAD_SETTLE:

            if ((HAL_GetTick() - state_tick) >= AGV_LOAD_SETTLE_MS)
            {
                AGV_Motion_SetBranch(
                    (cargo == LOAD_COLOR_RED) ? AGV_BRANCH_RED : AGV_BRANCH_BLUE);

                AGV_Motion_Start(&seg_to_end);
                Mission_SetState(MISSION_GO_END);
            }

            break;

        case MISSION_GO_END:

            if (motion == AGV_MOTION_LINE_END)
            {
                BSP_UART_Printf("DONE, END AT S=%ld mm\r\n",
                                (long)AGV_Motion_GetEndDistance());
                Mission_SetState(MISSION_DONE);
            }
            else if (motion == AGV_MOTION_FAULT)
            {
                BSP_UART_Printf("FAULT WHILE GO END\r\n");
                Mission_SetState(MISSION_FAULT);
            }

            break;

        case MISSION_DONE:
        case MISSION_FAULT:
        default:

            break;
    }

    Mission_Print();
}
