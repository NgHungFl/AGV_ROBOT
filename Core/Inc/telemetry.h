
#ifndef TELEMETRY_H
#define TELEMETRY_H

#include "main.h"
#include <stdint.h>

void Telemetry_SendLine(const uint16_t *data);

void Telemetry_SendColor(
    uint16_t red,
    uint16_t green,
    uint16_t blue,
    uint16_t clear,
    const char *color_name
);

#endif
