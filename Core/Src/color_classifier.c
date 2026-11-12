#include "color_classifier.h"

ColorType ColorClassifier_Classify(ColorData *data)
{
    if (data == NULL)
        return COLOR_UNKNOWN;

    uint32_t total = (uint32_t)data->red
                   + data->green
                   + data->blue;

    // Không đủ ánh sáng
    if (data->clear < 50 || total == 0)
        return COLOR_UNKNOWN;

    float red_ratio   = (float)data->red / total;
    float green_ratio = (float)data->green / total;
    float blue_ratio  = (float)data->blue / total;

    // Nhận diện màu đỏ
    if (red_ratio > 0.45f &&
        red_ratio > blue_ratio + 0.15f &&
        red_ratio > green_ratio + 0.10f)
    {
        return COLOR_RED;
    }

    // Nhận diện màu xanh dương
    if (blue_ratio > 0.45f &&
        blue_ratio > red_ratio + 0.15f &&
        blue_ratio > green_ratio + 0.10f)
    {
        return COLOR_BLUE;
    }

    return COLOR_UNKNOWN;
}