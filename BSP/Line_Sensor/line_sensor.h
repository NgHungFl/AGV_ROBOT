
#ifndef LINE_SENSOR_H
#define LINE_SENSOR_H

#include "main.h"
#include <stdint.h>

/* So luong mat do line */
#define LINE_SENSOR_COUNT 5U

/* Nguong phat hien line: 0 - 1000 */
#define LINE_DETECT_THRESHOLD 200U

/* =================================================
 * CALIBRATION
 * BLACK: Gia tri ADC khi dat tren vach den.
 * WHITE: Gia tri ADC khi dat tren nen trang.
 *
 * CAC GIA TRI BEN DUOI CHI LA VI DU.
 * PHAI DO THUC TE SAU KHI NAP CODE.
 * ================================================= */

#define LINE_S1_BLACK 145U
#define LINE_S1_WHITE  100U

#define LINE_S2_BLACK 145U
#define LINE_S2_WHITE  100U

#define LINE_S3_BLACK 150U
#define LINE_S3_WHITE  105U

#define LINE_S4_BLACK 170U
#define LINE_S4_WHITE  105U

#define LINE_S5_BLACK 190U
#define LINE_S5_WHITE  103U

/* Trong so tu trai sang phai */
#define LINE_WEIGHT_S1 (-2000)
#define LINE_WEIGHT_S2 (-1000)
#define LINE_WEIGHT_S3 (0)
#define LINE_WEIGHT_S4 (1000)
#define LINE_WEIGHT_S5 (2000)

/* Trang thai line */
typedef enum
{
    LINE_LOST = 0,
    LINE_DETECTED
} LineStatus_t;

/* Cau truc du lieu 5 mat */
typedef struct
{
    uint16_t raw[LINE_SENSOR_COUNT];

    uint16_t value[LINE_SENSOR_COUNT];

    int32_t position;

    LineStatus_t status;

} LineSensor_Data_t;

/* API */
HAL_StatusTypeDef LineSensor_Init(ADC_HandleTypeDef *hadc);

void LineSensor_Update(void);

const LineSensor_Data_t *LineSensor_GetData(void);

#endif
