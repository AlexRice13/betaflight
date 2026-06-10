#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "drivers/time.h"

#include "pg/pg.h"

#define STEP_RESPONSE_OSD_LABEL_LENGTH 3
#define STEP_RESPONSE_WORK_HZ 1000

typedef enum {
    STEP_RESPONSE_WINDOW_HANN = 0,
    STEP_RESPONSE_WINDOW_HAMMING,
    STEP_RESPONSE_WINDOW_RECTANGULAR,
    STEP_RESPONSE_WINDOW_COUNT
} stepResponseWindow_e;

typedef struct stepResponseConfig_s {
    uint16_t expectedHz;
    uint16_t windowMs;
    uint16_t responseMs;
    uint16_t minInput;
    uint16_t epsilon1e6;
    uint8_t windowType;
    uint8_t settleBandPercent;
    uint16_t settleHoldMs;
    uint8_t settleOutlierPercent;
    uint8_t minCorrPercent;
    uint16_t bandMinHz;
    uint16_t bandMaxHz;
    uint8_t armedOnly;
    char debugLabel[STEP_RESPONSE_OSD_LABEL_LENGTH + 1];
    char debug2Label[STEP_RESPONSE_OSD_LABEL_LENGTH + 1];
} stepResponseConfig_t;

PG_DECLARE(stepResponseConfig_t, stepResponseConfig);

void stepResponsePidLoopStart(timeUs_t currentTimeUs);
void stepResponseCapture(int axis, float setpoint, float gyroRate);
bool stepResponseUpdateCheck(timeUs_t currentTimeUs, timeDelta_t currentDeltaTimeUs);
void stepResponseUpdate(timeUs_t currentTimeUs);

const char *stepResponseGetDebugLabel(int row);
