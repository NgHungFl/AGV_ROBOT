
#ifndef LINE_SENSOR_H
#define LINE_SENSOR_H

#include "main.h"
#include <stdint.h>
#include "robot_config.h"

/* So luong mat do line */
#define LINE_SENSOR_COUNT 5U

/* Nguong phat hien line: 0 - 1000 */
#define LINE_DETECT_THRESHOLD 200U

/* So lan quet ADC lay trung binh moi lan cap nhat (DMA circular, 5*N mau).
   Nhieu giam ~ sqrt(N) lan. */
#ifndef LINE_OVERSAMPLE
#define LINE_OVERSAMPLE 16U
#endif

/* Trong so mem: mat co gia tri > FLOOR dong gop (value - FLOOR) vao vi tri,
   thay vi bat/tat o nguong => vi tri do lien tuc, khong nhay bac.
   LINE_SOFT_WEIGHT = 0: nguong cung nhu cu (>= 200 moi tinh). */
#ifndef LINE_SOFT_WEIGHT
#define LINE_SOFT_WEIGHT 1
#endif
#ifndef LINE_SOFT_FLOOR
#define LINE_SOFT_FLOOR 50U
#endif

/* =================================================
 * CALIBRATION
 * BLACK: Gia tri ADC khi dat tren vach den.
 * WHITE: Gia tri ADC khi dat tren nen trang.
 *
 * CAC GIA TRI BEN DUOI CHI LA VI DU.
 * PHAI DO THUC TE SAU KHI NAP CODE.
 * ================================================= */

#define LINE_S1_BLACK 148U
#define LINE_S1_WHITE 95U

#define LINE_S2_BLACK 147U
#define LINE_S2_WHITE 91U
 
#define LINE_S3_BLACK 153U
#define LINE_S3_WHITE 97U
 
#define LINE_S4_BLACK 148U
#define LINE_S4_WHITE 101U
 
#define LINE_S5_BLACK 173U
#define LINE_S5_WHITE 98U
 
/* Trong so tu trai sang phai */
#define LINE_WEIGHT_S1 (-2000)
#define LINE_WEIGHT_S2 (-1000)
#define LINE_WEIGHT_S3 (0)
#define LINE_WEIGHT_S4 (1000)
#define LINE_WEIGHT_S5 (2000)

/* Doi vi tri: 1000 don vi = 1 khoang cach giua 2 mat */
#define LINE_POSITION_TO_MM(pos) \
    ((float)(pos) * (ROBOT_SENSOR_PITCH_MM / 1000.0f))

/* Chon nhanh khi gap nga re:
   NONE  : lay trung binh tat ca mat thay line (nhu cu)
   LEFT  : chi bam nhom mat thay line o ben TRAI nhat
   RIGHT : chi bam nhom mat thay line o ben PHAI nhat */
typedef enum
{
    LINE_PREF_NONE = 0,
    LINE_PREF_LEFT,
    LINE_PREF_RIGHT
} LinePreference_t;

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

void LineSensor_SetPreference(LinePreference_t pref);

const LineSensor_Data_t *LineSensor_GetData(void);

#endif
