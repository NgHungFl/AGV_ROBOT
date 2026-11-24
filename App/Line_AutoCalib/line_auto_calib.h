#ifndef LINE_AUTO_CALIB_H
#define LINE_AUTO_CALIB_H

#include "main.h"

/* Call after LineSensor_Init() and before starting motors/control timer.
 * NRST -> sample stable WHITE -> sample stable BLACK patch -> save to Flash.
 * Power-cycle with valid Flash -> load stored values, no calibration.
 */
HAL_StatusTypeDef LineAutoCalib_Boot(uint8_t manual_nrst);

/* Disable driver and blink PC13 if calibration fails. */
void LineAutoCalib_ErrorBlink(void);

#endif
