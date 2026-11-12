#ifndef COLOR_CLASSIFIER_H
#define COLOR_CLASSIFIER_H

#include "color_sensor.h"

typedef enum
{
    COLOR_UNKNOWN = 0,
    COLOR_RED,
    COLOR_BLUE
} ColorType;

ColorType ColorClassifier_Classify(ColorData *data);

#endif