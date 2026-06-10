/*
 * Real-time setpoint/gyro step-response estimator for debug OSD.
 *
 * Welch-averaged spectral deconvolution (PIDtoolbox style):
 *
 *   H(f) = Pxy(f) / (Pxx(f) + epsilon)
 *
 * Capture runs at the analysis rate (default 1 kHz) inside the PID loop and
 * fills a double-buffered window (default 2 s). The background task slices
 * the window into overlapping 512-point segments (50% hop), accumulates the
 * auto/cross spectra, builds the regularised transfer function, recovers the
 * impulse response with one inverse RFFT and integrates it into a step
 * response. Rise time, overshoot and settling time are then measured on the
 * actual curve.
 *
 * Every task invocation is bounded to roughly one 512-point RFFT or less so
 * the TASK_PRIORITY_LOW task fits into the idle gaps of fast PID loops and
 * never starves or stalls the scheduler.
 *
 * Rise and settling times are published in 0.1 ms units (123 = 12.3 ms).
 */

#include <math.h>
#include <stdbool.h>
#include <float.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#include "build/debug.h"

#include "common/axis.h"
#include "common/maths.h"

#include "drivers/time.h"

#include "flight/step_response.h"

#include "fc/runtime_config.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#ifdef USE_STEP_RESPONSE_DEBUG

#ifndef UNIT_TEST
#include "arm_math.h"
#endif

#define STEP_RESPONSE_AXIS_COUNT 2
#define STEP_RESPONSE_REQUIRED_AXIS_MASK ((1U << 0) | (1U << 1))
#define STEP_RESPONSE_SIGNAL_COUNT 2
#define STEP_RESPONSE_SIGNAL_SETPOINT 0
#define STEP_RESPONSE_SIGNAL_GYRO 1

#define STEP_RESPONSE_MAX_ANALYSIS_HZ 1000
#define STEP_RESPONSE_MAX_WINDOW_MS 2000
#define STEP_RESPONSE_MAX_SAMPLES ((STEP_RESPONSE_MAX_ANALYSIS_HZ * STEP_RESPONSE_MAX_WINDOW_MS) / 1000)
#define STEP_RESPONSE_SAMPLE_ACCUMULATOR_SCALE 1000000U
#define STEP_RESPONSE_MAX_ASYNC_SAMPLES_PER_LOOP 16
#define STEP_RESPONSE_MAX_ASYNC_GAP_US 100000U

#define STEP_RESPONSE_FFT_SIZE 512
#define STEP_RESPONSE_HOP_SIZE (STEP_RESPONSE_FFT_SIZE / 2)
#define STEP_RESPONSE_BIN_COUNT ((STEP_RESPONSE_FFT_SIZE / 2) + 1)
// Only the first half of the circular impulse response is usable
#define STEP_RESPONSE_MAX_RESPONSE_SAMPLES (STEP_RESPONSE_FFT_SIZE / 2)
#define STEP_RESPONSE_SAMPLE_SCALE 4.0f

// Per-invocation batch sizes, chosen so each task call stays below ~100 us on an F7
#define STEP_RESPONSE_WINDOW_BATCH 64
#define STEP_RESPONSE_TRANSFER_BATCH 64
#define STEP_RESPONSE_SETTLE_BATCH 64

#define STEP_RESPONSE_DEFAULT_EPSILON_1E6 100
#define STEP_RESPONSE_DEFAULT_COHERENCE_PERCENT 30
#define STEP_RESPONSE_INVALID_METRIC -1
#define STEP_RESPONSE_MIN_STEADY_STATE 1.0e-3f
#define STEP_RESPONSE_QUALITY_EPSILON 1.0e-6f

PG_REGISTER_WITH_RESET_TEMPLATE(stepResponseConfig_t, stepResponseConfig, PG_STEP_RESPONSE_CONFIG, 2);

PG_RESET_TEMPLATE(stepResponseConfig_t, stepResponseConfig,
    .expectedHz = 1000,
    .windowMs = 2000,
    .responseMs = 250,
    .minInput = 20,
    .epsilon1e6 = STEP_RESPONSE_DEFAULT_EPSILON_1E6,
    .windowType = STEP_RESPONSE_WINDOW_HANN,
    .settleBandPercent = 20,
    .settleHoldMs = 80,
    .settleOutlierPercent = 10,
    .minCorrPercent = STEP_RESPONSE_DEFAULT_COHERENCE_PERCENT,
    .bandMinHz = 5,
    .bandMaxHz = 200,
    .armedOnly = 1,
    .debugLabel = "ROL",
    .debug2Label = "PIT"
);

typedef enum {
    STEP_RESPONSE_PHASE_IDLE = 0,
    STEP_RESPONSE_PHASE_WINDOW,
    STEP_RESPONSE_PHASE_SEGMENT,
    STEP_RESPONSE_PHASE_TRANSFER,
    STEP_RESPONSE_PHASE_IFFT,
    STEP_RESPONSE_PHASE_RESPONSE,
    STEP_RESPONSE_PHASE_SETTLE,
    STEP_RESPONSE_PHASE_PUBLISH
} stepResponsePhase_e;

typedef struct {
    int16_t riseTime01Ms;
    int16_t overshootPercent;
    int16_t settlingTime01Ms;
    int16_t qualityPercent;
} stepResponseMetrics_t;

typedef struct {
    bool rateValid;
    bool timeValid;
    uint8_t activeBuffer;
    uint8_t readyBuffer;
    uint16_t writeIndex;
    uint16_t sampleCount;
    uint16_t responseCount;
    uint16_t analysisHz;
    uint32_t sampleAccumulator;
    timeUs_t currentLoopTimeUs;
    timeUs_t lastSampleLoopTimeUs;
    uint8_t latestAxisMask;
    float latestSample[STEP_RESPONSE_AXIS_COUNT][STEP_RESPONSE_SIGNAL_COUNT];
    bool axisMetricsValid[STEP_RESPONSE_AXIS_COUNT];
    bool pendingJob;

    stepResponsePhase_e phase;
    uint8_t jobBuffer;
    uint8_t jobAxis;
    uint16_t phaseIndex;
    int16_t currentQuality;

    bool axisPrepared;
    uint8_t segmentStage;
    uint16_t segmentCount;
    float coherenceSum;
    uint16_t coherenceCount;

    float responsePeak;
    uint16_t settleHoldSamples;
    uint16_t settleAllowedOutliers;
    float settleBand;
    bool settleFound;
    uint16_t settleIndex;

    int8_t cachedWindowType;
    int8_t jobWindowType;
} stepResponseState_t;

static stepResponseState_t stepResponse = { .cachedWindowType = -1 };

static int16_t samples[2][STEP_RESPONSE_AXIS_COUNT][STEP_RESPONSE_SIGNAL_COUNT][STEP_RESPONSE_MAX_SAMPLES];
static float windowValues[STEP_RESPONSE_FFT_SIZE];
static float fftInput[STEP_RESPONSE_FFT_SIZE];
static float spectrumSetpoint[STEP_RESPONSE_FFT_SIZE];
static float spectrumGyro[STEP_RESPONSE_FFT_SIZE];
static float powerSetpoint[STEP_RESPONSE_BIN_COUNT];
static float powerGyro[STEP_RESPONSE_BIN_COUNT];
static float crossRe[STEP_RESPONSE_BIN_COUNT];
static float crossIm[STEP_RESPONSE_BIN_COUNT];
static float stepResponseValues[STEP_RESPONSE_MAX_RESPONSE_SAMPLES];

// ---------------------------------------------------------------------------
// FFT backend: CMSIS RFFT on target, portable radix-2 FFT for host unit tests
// ---------------------------------------------------------------------------

#ifdef UNIT_TEST

static float testFftRe[STEP_RESPONSE_FFT_SIZE];
static float testFftIm[STEP_RESPONSE_FFT_SIZE];

static void portableCfft(float *re, float *im, const uint16_t n, const bool inverse)
{
    for (uint16_t i = 1, j = 0; i < n; i++) {
        uint16_t bit = n >> 1;
        for (; j & bit; bit >>= 1) {
            j ^= bit;
        }
        j |= bit;

        if (i < j) {
            const float tmpRe = re[i];
            const float tmpIm = im[i];
            re[i] = re[j];
            im[i] = im[j];
            re[j] = tmpRe;
            im[j] = tmpIm;
        }
    }

    for (uint16_t len = 2; len <= n; len <<= 1) {
        const float angle = (inverse ? 2.0f : -2.0f) * M_PIf / len;
        const float stepRe = cosf(angle);
        const float stepIm = sinf(angle);

        for (uint16_t i = 0; i < n; i += len) {
            float wRe = 1.0f;
            float wIm = 0.0f;

            for (uint16_t k = 0; k < len / 2U; k++) {
                const uint16_t evenIndex = i + k;
                const uint16_t oddIndex = i + k + len / 2U;
                const float oddRe = re[oddIndex] * wRe - im[oddIndex] * wIm;
                const float oddIm = re[oddIndex] * wIm + im[oddIndex] * wRe;

                re[oddIndex] = re[evenIndex] - oddRe;
                im[oddIndex] = im[evenIndex] - oddIm;
                re[evenIndex] += oddRe;
                im[evenIndex] += oddIm;

                const float nextWRe = wRe * stepRe - wIm * stepIm;
                wIm = wRe * stepIm + wIm * stepRe;
                wRe = nextWRe;
            }
        }
    }

    if (inverse) {
        for (uint16_t i = 0; i < n; i++) {
            re[i] /= n;
            im[i] /= n;
        }
    }
}

static bool stepResponseFftInit(void)
{
    return true;
}

// Produces CMSIS rfft_fast packing: [DC, Nyquist, Re1, Im1, Re2, Im2, ...]
static void stepResponseFftForward(float *input, float *spectrum)
{
    for (uint16_t i = 0; i < STEP_RESPONSE_FFT_SIZE; i++) {
        testFftRe[i] = input[i];
        testFftIm[i] = 0.0f;
    }

    portableCfft(testFftRe, testFftIm, STEP_RESPONSE_FFT_SIZE, false);

    spectrum[0] = testFftRe[0];
    spectrum[1] = testFftRe[STEP_RESPONSE_FFT_SIZE / 2];

    for (uint16_t bin = 1; bin < STEP_RESPONSE_FFT_SIZE / 2U; bin++) {
        spectrum[2U * bin] = testFftRe[bin];
        spectrum[(2U * bin) + 1U] = testFftIm[bin];
    }
}

static void stepResponseFftInverse(float *spectrum, float *output)
{
    testFftRe[0] = spectrum[0];
    testFftIm[0] = 0.0f;
    testFftRe[STEP_RESPONSE_FFT_SIZE / 2] = spectrum[1];
    testFftIm[STEP_RESPONSE_FFT_SIZE / 2] = 0.0f;

    for (uint16_t bin = 1; bin < STEP_RESPONSE_FFT_SIZE / 2U; bin++) {
        const float re = spectrum[2U * bin];
        const float im = spectrum[(2U * bin) + 1U];

        testFftRe[bin] = re;
        testFftIm[bin] = im;
        testFftRe[STEP_RESPONSE_FFT_SIZE - bin] = re;
        testFftIm[STEP_RESPONSE_FFT_SIZE - bin] = -im;
    }

    portableCfft(testFftRe, testFftIm, STEP_RESPONSE_FFT_SIZE, true);

    for (uint16_t i = 0; i < STEP_RESPONSE_FFT_SIZE; i++) {
        output[i] = testFftRe[i];
    }
}

#else

static arm_rfft_fast_instance_f32 rfftInstance;
static bool rfftInitialised;

static bool stepResponseFftInit(void)
{
    if (rfftInitialised) {
        return true;
    }

    rfftInitialised = arm_rfft_fast_init_f32(&rfftInstance, STEP_RESPONSE_FFT_SIZE) == ARM_MATH_SUCCESS;
    return rfftInitialised;
}

static void stepResponseFftForward(float *input, float *spectrum)
{
    arm_rfft_fast_f32(&rfftInstance, input, spectrum, 0);
}

static void stepResponseFftInverse(float *spectrum, float *output)
{
    arm_rfft_fast_f32(&rfftInstance, spectrum, output, 1);
}

#endif // UNIT_TEST

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static int16_t constrainDebugValue(const int value)
{
    return constrain(value, INT16_MIN, INT16_MAX);
}

static uint16_t clampNonZeroU16(const uint16_t value, const uint16_t minValue, const uint16_t maxValue)
{
    return constrain(value ? value : minValue, minValue, maxValue);
}

static void setAxisQualityOnly(const uint8_t axisIndex, const int16_t qualityPercent)
{
    const int base = axisIndex * 4;

    if (!stepResponse.axisMetricsValid[axisIndex]) {
        DEBUG_SET(DEBUG_STEP_RESPONSE, base + 0, 0);
        DEBUG_SET(DEBUG_STEP_RESPONSE, base + 1, 0);
        DEBUG_SET(DEBUG_STEP_RESPONSE, base + 2, 0);
    }

    DEBUG_SET(DEBUG_STEP_RESPONSE, base + 3, qualityPercent);
}

static void publishInvalidMetrics(void)
{
    for (int axis = 0; axis < STEP_RESPONSE_AXIS_COUNT; axis++) {
        setAxisQualityOnly(axis, 0);
    }
}

static void setAxisMetrics(const uint8_t axisIndex, const stepResponseMetrics_t *metrics)
{
    const int base = axisIndex * 4;

    if (metrics->riseTime01Ms < 0 || metrics->overshootPercent < 0 || metrics->settlingTime01Ms < 0) {
        setAxisQualityOnly(axisIndex, metrics->qualityPercent);
        return;
    }

    DEBUG_SET(DEBUG_STEP_RESPONSE, base + 0, metrics->riseTime01Ms);
    DEBUG_SET(DEBUG_STEP_RESPONSE, base + 1, metrics->overshootPercent);
    DEBUG_SET(DEBUG_STEP_RESPONSE, base + 2, metrics->settlingTime01Ms);
    DEBUG_SET(DEBUG_STEP_RESPONSE, base + 3, metrics->qualityPercent);

    stepResponse.axisMetricsValid[axisIndex] = true;
}

static void setAxisInvalidMetrics(const uint8_t axisIndex, const int16_t qualityPercent)
{
    setAxisQualityOnly(axisIndex, qualityPercent);
}

// ---------------------------------------------------------------------------
// Capture (PID loop side)
// ---------------------------------------------------------------------------

static void resetCaptureState(void)
{
    stepResponse.rateValid = false;
    stepResponse.timeValid = false;
    stepResponse.writeIndex = 0;
    stepResponse.sampleAccumulator = 0;
    stepResponse.latestAxisMask = 0;
}

static bool configureCaptureState(void)
{
    const stepResponseConfig_t *config = stepResponseConfig();
    const uint16_t analysisHz = clampNonZeroU16(config->expectedHz, 1, STEP_RESPONSE_MAX_ANALYSIS_HZ);
    const uint16_t windowMs = clampNonZeroU16(config->windowMs, 100, STEP_RESPONSE_MAX_WINDOW_MS);
    const uint16_t responseMs = clampNonZeroU16(config->responseMs, 10, 500);
    const uint32_t sampleCount = ((uint32_t)analysisHz * windowMs) / 1000U;
    const uint32_t responseCount = MIN(((uint32_t)analysisHz * responseMs) / 1000U, (uint32_t)STEP_RESPONSE_MAX_RESPONSE_SAMPLES);

    // The Welch pipeline needs at least one full FFT segment
    if (!responseCount || sampleCount < STEP_RESPONSE_FFT_SIZE || sampleCount > STEP_RESPONSE_MAX_SAMPLES) {
        resetCaptureState();
        publishInvalidMetrics();
        return false;
    }

    if (stepResponse.analysisHz != analysisHz ||
        stepResponse.sampleCount != sampleCount || stepResponse.responseCount != responseCount) {
        stepResponse.writeIndex = 0;
        stepResponse.timeValid = false;
        stepResponse.sampleAccumulator = 0;
        stepResponse.latestAxisMask = 0;
        stepResponse.pendingJob = false;
        stepResponse.phase = STEP_RESPONSE_PHASE_IDLE;
    }

    stepResponse.analysisHz = analysisHz;
    stepResponse.sampleCount = sampleCount;
    stepResponse.responseCount = responseCount;
    stepResponse.rateValid = true;

    return true;
}

void stepResponsePidLoopStart(const timeUs_t currentTimeUs)
{
    stepResponse.currentLoopTimeUs = currentTimeUs;
    stepResponse.latestAxisMask = 0;

    if (debugMode != DEBUG_STEP_RESPONSE) {
        resetCaptureState();
        return;
    }

    if (stepResponseConfig()->armedOnly && !ARMING_FLAG(ARMED)) {
        resetCaptureState();
        publishInvalidMetrics();
        return;
    }

    if (!configureCaptureState()) {
        return;
    }
}

static int stepResponseAxisIndex(const int axis)
{
    switch (axis) {
    case FD_ROLL:
        return 0;
    case FD_PITCH:
        return 1;
    default:
        return -1;
    }
}

static void finishCapturedSample(void)
{
    if (++stepResponse.writeIndex < stepResponse.sampleCount) {
        return;
    }

    stepResponse.writeIndex = 0;

    if (stepResponse.pendingJob || stepResponse.phase != STEP_RESPONSE_PHASE_IDLE) {
        // Analysis still busy: drop this window silently and keep the last
        // published metrics on the OSD instead of stomping them with zeros.
        return;
    }

    stepResponse.readyBuffer = stepResponse.activeBuffer;
    stepResponse.pendingJob = true;
    stepResponse.activeBuffer ^= 1U;
}

static float getCapturedSample(const uint8_t axisIndex, const uint8_t signalIndex, const uint16_t sampleIndex)
{
    return samples[stepResponse.jobBuffer][axisIndex][signalIndex][sampleIndex] / STEP_RESPONSE_SAMPLE_SCALE;
}

static void storeCapturedSample(const uint8_t bufferIndex, const uint8_t axisIndex, const uint8_t signalIndex, const uint16_t sampleIndex, const float value)
{
    samples[bufferIndex][axisIndex][signalIndex][sampleIndex] = constrain(lrintf(value * STEP_RESPONSE_SAMPLE_SCALE), INT16_MIN, INT16_MAX);
}

static void storeLatestCapturedSample(void)
{
    const uint16_t sampleIndex = stepResponse.writeIndex;
    const uint8_t bufferIndex = stepResponse.activeBuffer;

    for (uint8_t axisIndex = 0; axisIndex < STEP_RESPONSE_AXIS_COUNT; axisIndex++) {
        storeCapturedSample(bufferIndex, axisIndex, STEP_RESPONSE_SIGNAL_SETPOINT, sampleIndex, stepResponse.latestSample[axisIndex][STEP_RESPONSE_SIGNAL_SETPOINT]);
        storeCapturedSample(bufferIndex, axisIndex, STEP_RESPONSE_SIGNAL_GYRO, sampleIndex, stepResponse.latestSample[axisIndex][STEP_RESPONSE_SIGNAL_GYRO]);
    }

    finishCapturedSample();
}

STATIC_UNIT_TESTED uint8_t stepResponseUpdateSampleAccumulator(uint32_t *sampleAccumulator, const uint32_t deltaUs, const uint16_t analysisHz)
{
    uint64_t accumulator = *sampleAccumulator + (uint64_t)deltaUs * analysisHz;
    uint8_t dueSamples = 0;

    while (accumulator >= STEP_RESPONSE_SAMPLE_ACCUMULATOR_SCALE && dueSamples < STEP_RESPONSE_MAX_ASYNC_SAMPLES_PER_LOOP) {
        accumulator -= STEP_RESPONSE_SAMPLE_ACCUMULATOR_SCALE;
        dueSamples++;
    }

    if (accumulator >= STEP_RESPONSE_SAMPLE_ACCUMULATOR_SCALE) {
        accumulator %= STEP_RESPONSE_SAMPLE_ACCUMULATOR_SCALE;
    }

    *sampleAccumulator = (uint32_t)accumulator;

    return dueSamples;
}

static void captureDueSamples(void)
{
    if (!stepResponse.timeValid) {
        stepResponse.timeValid = true;
        stepResponse.lastSampleLoopTimeUs = stepResponse.currentLoopTimeUs;
        stepResponse.sampleAccumulator = 0;
        return;
    }

    const timeDelta_t deltaUs = cmpTimeUs(stepResponse.currentLoopTimeUs, stepResponse.lastSampleLoopTimeUs);
    stepResponse.lastSampleLoopTimeUs = stepResponse.currentLoopTimeUs;

    if (deltaUs <= 0) {
        return;
    }

    if ((uint32_t)deltaUs > STEP_RESPONSE_MAX_ASYNC_GAP_US) {
        stepResponse.sampleAccumulator = 0;
        return;
    }

    uint8_t dueSamples = stepResponseUpdateSampleAccumulator(&stepResponse.sampleAccumulator, (uint32_t)deltaUs, stepResponse.analysisHz);

    while (dueSamples--) {
        storeLatestCapturedSample();
    }
}

void stepResponseCapture(const int axis, const float setpoint, const float gyroRate)
{
    if (!stepResponse.rateValid) {
        return;
    }

    const int axisIndex = stepResponseAxisIndex(axis);
    if (axisIndex < 0) {
        return;
    }

    stepResponse.latestSample[axisIndex][STEP_RESPONSE_SIGNAL_SETPOINT] = setpoint;
    stepResponse.latestSample[axisIndex][STEP_RESPONSE_SIGNAL_GYRO] = gyroRate;
    stepResponse.latestAxisMask |= (1U << axisIndex);

    if ((stepResponse.latestAxisMask & STEP_RESPONSE_REQUIRED_AXIS_MASK) == STEP_RESPONSE_REQUIRED_AXIS_MASK) {
        captureDueSamples();
    }
}

// ---------------------------------------------------------------------------
// Background analysis pipeline
// ---------------------------------------------------------------------------

static void advanceAxisOrFinishJob(void)
{
    if (++stepResponse.jobAxis >= STEP_RESPONSE_AXIS_COUNT) {
        stepResponse.phase = STEP_RESPONSE_PHASE_IDLE;
        stepResponse.pendingJob = false;
        return;
    }

    stepResponse.phaseIndex = 0;
    stepResponse.segmentStage = 0;
    stepResponse.axisPrepared = false;
    stepResponse.currentQuality = 0;
    stepResponse.phase = STEP_RESPONSE_PHASE_SEGMENT;
}

static void failAxisAndAdvance(const int16_t qualityPercent)
{
    setAxisInvalidMetrics(stepResponse.jobAxis, qualityPercent);
    advanceAxisOrFinishJob();
}

static float getWindowValue(const uint16_t sampleIndex)
{
    const float phase = 2.0f * M_PIf * sampleIndex / (STEP_RESPONSE_FFT_SIZE - 1);

    switch (stepResponse.jobWindowType) {
    case STEP_RESPONSE_WINDOW_HAMMING:
        return 0.54f - 0.46f * cosf(phase);
    case STEP_RESPONSE_WINDOW_RECTANGULAR:
        return 1.0f;
    case STEP_RESPONSE_WINDOW_HANN:
    default:
        return 0.5f - 0.5f * cosf(phase);
    }
}

static void processWindowBatch(void)
{
    const uint16_t endIndex = MIN(stepResponse.phaseIndex + STEP_RESPONSE_WINDOW_BATCH, STEP_RESPONSE_FFT_SIZE);

    for (uint16_t i = stepResponse.phaseIndex; i < endIndex; i++) {
        windowValues[i] = getWindowValue(i);
    }

    stepResponse.phaseIndex = endIndex;

    if (stepResponse.phaseIndex >= STEP_RESPONSE_FFT_SIZE) {
        stepResponse.cachedWindowType = stepResponse.jobWindowType;
        stepResponse.phaseIndex = 0;
        stepResponse.segmentStage = 0;
        stepResponse.axisPrepared = false;
        stepResponse.phase = STEP_RESPONSE_PHASE_SEGMENT;
    }
}

static uint16_t getSegmentCount(void)
{
    if (stepResponse.sampleCount < STEP_RESPONSE_FFT_SIZE) {
        return 0;
    }

    return 1 + ((stepResponse.sampleCount - STEP_RESPONSE_FFT_SIZE) / STEP_RESPONSE_HOP_SIZE);
}

// Window-level input gate, mirrors PIDtoolbox minInput segment selection
static bool axisInputIsUsable(const uint8_t axisIndex)
{
    const stepResponseConfig_t *config = stepResponseConfig();
    float setpointMin = getCapturedSample(axisIndex, STEP_RESPONSE_SIGNAL_SETPOINT, 0);
    float setpointMax = setpointMin;
    float setpointSum = 0.0f;

    for (uint16_t i = 0; i < stepResponse.sampleCount; i++) {
        const float setpoint = getCapturedSample(axisIndex, STEP_RESPONSE_SIGNAL_SETPOINT, i);

        setpointMin = fminf(setpointMin, setpoint);
        setpointMax = fmaxf(setpointMax, setpoint);
        setpointSum += setpoint;
    }

    if (config->minInput && (setpointMax - setpointMin) < config->minInput) {
        return false;
    }

    const float setpointMean = setpointSum / stepResponse.sampleCount;
    float setpointEnergy = 0.0f;

    for (uint16_t i = 0; i < stepResponse.sampleCount; i++) {
        const float setpoint = getCapturedSample(axisIndex, STEP_RESPONSE_SIGNAL_SETPOINT, i) - setpointMean;
        setpointEnergy += setpoint * setpoint;
    }

    return setpointEnergy >= STEP_RESPONSE_QUALITY_EPSILON;
}

static bool prepareAxis(void)
{
    stepResponse.currentQuality = 0;
    stepResponse.segmentCount = getSegmentCount();

    if (!stepResponseFftInit() || !stepResponse.segmentCount || !axisInputIsUsable(stepResponse.jobAxis)) {
        return false;
    }

    memset(powerSetpoint, 0, sizeof(powerSetpoint));
    memset(powerGyro, 0, sizeof(powerGyro));
    memset(crossRe, 0, sizeof(crossRe));
    memset(crossIm, 0, sizeof(crossIm));

    return true;
}

static void fillFftInput(const uint8_t axisIndex, const uint8_t signalIndex, const uint16_t startIndex)
{
    for (uint16_t i = 0; i < STEP_RESPONSE_FFT_SIZE; i++) {
        fftInput[i] = getCapturedSample(axisIndex, signalIndex, startIndex + i) * windowValues[i];
    }
}

static void getSpectrumBin(const float *spectrum, const uint16_t bin, float *re, float *im)
{
    if (bin == 0) {
        *re = spectrum[0];
        *im = 0.0f;
    } else if (bin == STEP_RESPONSE_FFT_SIZE / 2) {
        *re = spectrum[1];
        *im = 0.0f;
    } else {
        *re = spectrum[2U * bin];
        *im = spectrum[(2U * bin) + 1U];
    }
}

static void setTransferSpectrumBin(const uint16_t bin, const float re, const float im)
{
    if (bin == 0) {
        spectrumSetpoint[0] = re;
    } else if (bin == STEP_RESPONSE_FFT_SIZE / 2) {
        spectrumSetpoint[1] = re;
    } else {
        spectrumSetpoint[2U * bin] = re;
        spectrumSetpoint[(2U * bin) + 1U] = im;
    }
}

static void accumulateSegmentSpectra(void)
{
    for (uint16_t bin = 0; bin < STEP_RESPONSE_BIN_COUNT; bin++) {
        float setpointRe;
        float setpointIm;
        float gyroRe;
        float gyroIm;

        getSpectrumBin(spectrumSetpoint, bin, &setpointRe, &setpointIm);
        getSpectrumBin(spectrumGyro, bin, &gyroRe, &gyroIm);

        powerSetpoint[bin] += setpointRe * setpointRe + setpointIm * setpointIm;
        powerGyro[bin] += gyroRe * gyroRe + gyroIm * gyroIm;
        crossRe[bin] += gyroRe * setpointRe + gyroIm * setpointIm;
        crossIm[bin] += gyroIm * setpointRe - gyroRe * setpointIm;
    }
}

static void processSegmentBatch(void)
{
    if (!stepResponse.axisPrepared) {
        if (!prepareAxis()) {
            failAxisAndAdvance(0);
            return;
        }

        stepResponse.axisPrepared = true;
        return;
    }

    const uint16_t segmentStart = stepResponse.phaseIndex * STEP_RESPONSE_HOP_SIZE;

    if (stepResponse.segmentStage == 0) {
        fillFftInput(stepResponse.jobAxis, STEP_RESPONSE_SIGNAL_SETPOINT, segmentStart);
        stepResponseFftForward(fftInput, spectrumSetpoint);
        stepResponse.segmentStage = 1;
        return;
    }

    fillFftInput(stepResponse.jobAxis, STEP_RESPONSE_SIGNAL_GYRO, segmentStart);
    stepResponseFftForward(fftInput, spectrumGyro);
    accumulateSegmentSpectra();

    stepResponse.segmentStage = 0;
    stepResponse.phaseIndex++;

    if (stepResponse.phaseIndex >= stepResponse.segmentCount) {
        stepResponse.phaseIndex = 0;
        stepResponse.coherenceSum = 0.0f;
        stepResponse.coherenceCount = 0;
        memset(spectrumSetpoint, 0, sizeof(spectrumSetpoint));
        stepResponse.phase = STEP_RESPONSE_PHASE_TRANSFER;
    }
}

static void getBandLimits(uint16_t *minHz, uint16_t *maxHz)
{
    const stepResponseConfig_t *config = stepResponseConfig();
    const uint16_t nyquistHz = stepResponse.analysisHz / 2U;

    *minHz = clampNonZeroU16(config->bandMinHz, 1, nyquistHz);
    *maxHz = clampNonZeroU16(config->bandMaxHz, *minHz, nyquistHz);

    if (*maxHz < *minHz) {
        *maxHz = *minHz;
    }
}

static float getBinHz(const uint16_t bin)
{
    return (float)bin * stepResponse.analysisHz / STEP_RESPONSE_FFT_SIZE;
}

static void processTransferBatch(void)
{
    const stepResponseConfig_t *config = stepResponseConfig();
    const float epsilon = config->epsilon1e6 * 1.0e-6f;
    const uint16_t endBin = MIN(stepResponse.phaseIndex + STEP_RESPONSE_TRANSFER_BATCH, STEP_RESPONSE_BIN_COUNT);
    uint16_t minHz;
    uint16_t maxHz;

    getBandLimits(&minHz, &maxHz);

    for (uint16_t bin = stepResponse.phaseIndex; bin < endBin; bin++) {
        const float pxx = powerSetpoint[bin];

        if (pxx <= STEP_RESPONSE_QUALITY_EPSILON) {
            continue;
        }

        const float denominator = pxx + epsilon;
        const float hRe = crossRe[bin] / denominator;
        const float hIm = crossIm[bin] / denominator;
        const float frequencyHz = getBinHz(bin);

        // Truncate the transfer function above the band of interest to keep
        // high-frequency noise out of the reconstructed step response
        if (frequencyHz <= maxHz) {
            setTransferSpectrumBin(bin, hRe, hIm);
        }

        if (frequencyHz >= minHz && frequencyHz <= maxHz) {
            const float pyy = powerGyro[bin];

            if (pyy > STEP_RESPONSE_QUALITY_EPSILON) {
                const float crossMagSq = crossRe[bin] * crossRe[bin] + crossIm[bin] * crossIm[bin];
                stepResponse.coherenceSum += constrainf(crossMagSq / (pxx * pyy), 0.0f, 1.0f);
                stepResponse.coherenceCount++;
            }
        }
    }

    stepResponse.phaseIndex = endBin;

    if (stepResponse.phaseIndex >= STEP_RESPONSE_BIN_COUNT) {
        if (!stepResponse.coherenceCount) {
            failAxisAndAdvance(0);
            return;
        }

        const int16_t qualityPercent = constrainDebugValue(lrintf(constrainf(stepResponse.coherenceSum / stepResponse.coherenceCount, 0.0f, 1.0f) * 100.0f));
        stepResponse.currentQuality = qualityPercent;

        if (qualityPercent < config->minCorrPercent) {
            failAxisAndAdvance(qualityPercent);
            return;
        }

        stepResponse.phaseIndex = 0;
        stepResponse.phase = STEP_RESPONSE_PHASE_IFFT;
    }
}

static void processInverseFft(void)
{
    stepResponseFftInverse(spectrumSetpoint, fftInput);

    stepResponse.phaseIndex = 0;
    stepResponse.phase = STEP_RESPONSE_PHASE_RESPONSE;
}

static void processResponse(void)
{
    const stepResponseConfig_t *config = stepResponseConfig();
    const uint16_t responseCount = stepResponse.responseCount;
    float cumulativeImpulse = 0.0f;

    for (uint16_t i = 0; i < responseCount; i++) {
        cumulativeImpulse += fftInput[i];
        stepResponseValues[i] = cumulativeImpulse;
    }

    const uint16_t steadySamples = MAX(1U, responseCount / 5U);
    const uint16_t steadyStart = responseCount - steadySamples;
    const float baseline = stepResponseValues[0];
    float steadySum = 0.0f;

    for (uint16_t i = steadyStart; i < responseCount; i++) {
        steadySum += stepResponseValues[i] - baseline;
    }

    const float steadyState = steadySum / steadySamples;
    if (fabsf(steadyState) < STEP_RESPONSE_MIN_STEADY_STATE) {
        failAxisAndAdvance(stepResponse.currentQuality);
        return;
    }

    float peak = -FLT_MAX;

    for (uint16_t i = 0; i < responseCount; i++) {
        stepResponseValues[i] = (stepResponseValues[i] - baseline) / steadyState;
        peak = fmaxf(peak, stepResponseValues[i]);
    }

    stepResponse.responsePeak = peak;

    uint16_t holdSamples = ((uint32_t)config->settleHoldMs * stepResponse.analysisHz) / 1000U;
    holdSamples = clampNonZeroU16(holdSamples, 1, responseCount);

    stepResponse.settleHoldSamples = holdSamples;
    stepResponse.settleAllowedOutliers = ((uint32_t)holdSamples * config->settleOutlierPercent) / 100U;
    stepResponse.settleBand = config->settleBandPercent * 0.01f;
    stepResponse.settleFound = false;
    stepResponse.settleIndex = 0;

    stepResponse.phaseIndex = 0;
    stepResponse.phase = STEP_RESPONSE_PHASE_SETTLE;
}

static bool settlingWindowIsStable(const uint16_t startIndex)
{
    const float band = stepResponse.settleBand;
    uint16_t outliers = 0;
    float sum = 0.0f;

    for (uint16_t i = 0; i < stepResponse.settleHoldSamples; i++) {
        const float value = stepResponseValues[startIndex + i];
        sum += value;
        if (fabsf(value - 1.0f) > band) {
            outliers++;
        }
    }

    const float mean = sum / stepResponse.settleHoldSamples;
    return outliers <= stepResponse.settleAllowedOutliers && fabsf(mean - 1.0f) <= band;
}

static void processSettleBatch(void)
{
    const uint16_t lastStart = stepResponse.responseCount - stepResponse.settleHoldSamples;
    const uint16_t endIndex = MIN(stepResponse.phaseIndex + STEP_RESPONSE_SETTLE_BATCH, lastStart + 1U);

    for (uint16_t i = stepResponse.phaseIndex; i < endIndex; i++) {
        if (settlingWindowIsStable(i)) {
            stepResponse.settleFound = true;
            stepResponse.settleIndex = i;
            stepResponse.phase = STEP_RESPONSE_PHASE_PUBLISH;
            return;
        }
    }

    stepResponse.phaseIndex = endIndex;

    if (stepResponse.phaseIndex > lastStart) {
        stepResponse.phase = STEP_RESPONSE_PHASE_PUBLISH;
    }
}

static float findCrossingSample(const float level)
{
    for (uint16_t i = 1; i < stepResponse.responseCount; i++) {
        const float previous = stepResponseValues[i - 1U];
        const float current = stepResponseValues[i];

        if (previous < level && current >= level) {
            const float denominator = current - previous;
            if (fabsf(denominator) < 1.0e-6f) {
                return i;
            }

            return (i - 1U) + (level - previous) / denominator;
        }
    }

    return -1.0f;
}

// Converts a (fractional) sample offset to 0.1 ms units
static int sampleToTenthMs(const float sample)
{
    return lrintf(sample * 10000.0f / stepResponse.analysisHz);
}

static void processPublish(void)
{
    const float riseStart = findCrossingSample(0.1f);
    const float riseEnd = findCrossingSample(0.9f);
    const int riseTime01Ms = (riseStart >= 0.0f && riseEnd >= riseStart) ? sampleToTenthMs(riseEnd - riseStart) : STEP_RESPONSE_INVALID_METRIC;
    const int overshootPercent = lrintf(fmaxf(0.0f, stepResponse.responsePeak - 1.0f) * 100.0f);
    const int settlingTime01Ms = stepResponse.settleFound ? sampleToTenthMs(stepResponse.settleIndex) : STEP_RESPONSE_INVALID_METRIC;

    const stepResponseMetrics_t metrics = {
        .riseTime01Ms = constrainDebugValue(riseTime01Ms),
        .overshootPercent = constrainDebugValue(overshootPercent),
        .settlingTime01Ms = constrainDebugValue(settlingTime01Ms),
        .qualityPercent = stepResponse.currentQuality
    };

    setAxisMetrics(stepResponse.jobAxis, &metrics);
    advanceAxisOrFinishJob();
}

// ---------------------------------------------------------------------------
// Task entry points
// ---------------------------------------------------------------------------

bool stepResponseUpdateCheck(const timeUs_t currentTimeUs, const timeDelta_t currentDeltaTimeUs)
{
    UNUSED(currentTimeUs);
    UNUSED(currentDeltaTimeUs);

    if (debugMode != DEBUG_STEP_RESPONSE) {
        return false;
    }

    return stepResponse.pendingJob || stepResponse.phase != STEP_RESPONSE_PHASE_IDLE;
}

void stepResponseUpdate(const timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);

    if (debugMode != DEBUG_STEP_RESPONSE) {
        return;
    }

    if (stepResponse.phase == STEP_RESPONSE_PHASE_IDLE) {
        if (!stepResponse.pendingJob) {
            return;
        }

        stepResponse.jobBuffer = stepResponse.readyBuffer;
        stepResponse.jobAxis = 0;
        stepResponse.phaseIndex = 0;
        stepResponse.segmentStage = 0;
        stepResponse.axisPrepared = false;
        stepResponse.currentQuality = 0;
        stepResponse.jobWindowType = stepResponseConfig()->windowType;

        if (stepResponse.cachedWindowType == stepResponse.jobWindowType) {
            stepResponse.phase = STEP_RESPONSE_PHASE_SEGMENT;
        } else {
            stepResponse.phase = STEP_RESPONSE_PHASE_WINDOW;
        }
    }

    switch (stepResponse.phase) {
    case STEP_RESPONSE_PHASE_WINDOW:
        processWindowBatch();
        break;
    case STEP_RESPONSE_PHASE_SEGMENT:
        processSegmentBatch();
        break;
    case STEP_RESPONSE_PHASE_TRANSFER:
        processTransferBatch();
        break;
    case STEP_RESPONSE_PHASE_IFFT:
        processInverseFft();
        break;
    case STEP_RESPONSE_PHASE_RESPONSE:
        processResponse();
        break;
    case STEP_RESPONSE_PHASE_SETTLE:
        processSettleBatch();
        break;
    case STEP_RESPONSE_PHASE_PUBLISH:
        processPublish();
        break;
    case STEP_RESPONSE_PHASE_IDLE:
    default:
        break;
    }
}

const char *stepResponseGetDebugLabel(const int row)
{
    const char *label = row ? stepResponseConfig()->debug2Label : stepResponseConfig()->debugLabel;
    const char *fallback = row ? "D2" : "DBG";

    return label[0] ? label : fallback;
}

// ---------------------------------------------------------------------------
// Unit test helpers
// ---------------------------------------------------------------------------

#ifdef UNIT_TEST
void stepResponseTestReset(void)
{
    memset(&stepResponse, 0, sizeof(stepResponse));
    memset(samples, 0, sizeof(samples));
    memset(stepResponseValues, 0, sizeof(stepResponseValues));
    stepResponse.cachedWindowType = -1;
}

uint16_t stepResponseTestGetWriteIndex(void)
{
    return stepResponse.writeIndex;
}

uint16_t stepResponseTestGetAnalysisHz(void)
{
    return stepResponse.analysisHz;
}

uint16_t stepResponseTestGetSampleCount(void)
{
    return stepResponse.sampleCount;
}

uint16_t stepResponseTestGetResponseCount(void)
{
    return stepResponse.responseCount;
}

uint8_t stepResponseTestGetActiveBuffer(void)
{
    return stepResponse.activeBuffer;
}

bool stepResponseTestJobBusy(void)
{
    return stepResponse.pendingJob || stepResponse.phase != STEP_RESPONSE_PHASE_IDLE;
}

float stepResponseTestGetSample(const uint8_t bufferIndex, const uint8_t axisIndex, const uint8_t signalIndex, const uint16_t sampleIndex)
{
    return samples[bufferIndex][axisIndex][signalIndex][sampleIndex] / STEP_RESPONSE_SAMPLE_SCALE;
}

void stepResponseTestSetAxisMetrics(const uint8_t axisIndex, const int16_t riseTime01Ms, const int16_t overshootPercent, const int16_t settlingTime01Ms, const int16_t qualityPercent)
{
    const stepResponseMetrics_t metrics = {
        .riseTime01Ms = riseTime01Ms,
        .overshootPercent = overshootPercent,
        .settlingTime01Ms = settlingTime01Ms,
        .qualityPercent = qualityPercent
    };

    setAxisMetrics(axisIndex, &metrics);
}

void stepResponseTestSetAxisQualityOnly(const uint8_t axisIndex, const int16_t qualityPercent)
{
    setAxisQualityOnly(axisIndex, qualityPercent);
}
#endif

#else

void stepResponsePidLoopStart(const timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);
}

void stepResponseCapture(const int axis, const float setpoint, const float gyroRate)
{
    UNUSED(axis);
    UNUSED(setpoint);
    UNUSED(gyroRate);
}

bool stepResponseUpdateCheck(const timeUs_t currentTimeUs, const timeDelta_t currentDeltaTimeUs)
{
    UNUSED(currentTimeUs);
    UNUSED(currentDeltaTimeUs);

    return false;
}

void stepResponseUpdate(const timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);
}

const char *stepResponseGetDebugLabel(const int row)
{
    return row ? "D2" : "DBG";
}

#endif // USE_STEP_RESPONSE_DEBUG
