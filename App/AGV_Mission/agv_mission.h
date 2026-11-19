
#ifndef AGV_MISSION_H
#define AGV_MISSION_H

#include "main.h"

typedef enum
{
    MISSION_WAIT_START = 0,
    MISSION_GO_LOAD,
    MISSION_WAIT_LOAD,
    MISSION_LOAD_SETTLE,
    MISSION_GO_END,
    MISSION_DONE,
    MISSION_FAULT
} MissionState_t;

HAL_StatusTypeDef AGV_Mission_Init(I2C_HandleTypeDef *hi2c);

/* Goi lien tuc trong while(1) cua main */
void AGV_Mission_Run(void);

MissionState_t AGV_Mission_GetState(void);

#endif /* AGV_MISSION_H */
