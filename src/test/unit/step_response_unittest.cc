/*
 * Real-time step-response estimator tests.
 *
 * Covers the capture/decimation path, the Welch FFT deconvolution pipeline
 * (using the portable host-side FFT backend) and the OSD metric publishing
 * rules, including the 0.1 ms metric scaling.
 */

#include <math.h>
#include <stdint.h>
#include <string.h>

extern "C" {
    #include "platform.h"

    #include "build/debug.h"

    #include "common/axis.h"

    #include "fc/runtime_config.h"

    #include "flight/step_response.h"

    extern const stepResponseConfig_t pgResetTemplate_stepResponseConfig;

    uint8_t stepResponseUpdateSampleAccumulator(uint32_t *sampleAccumulator, uint32_t deltaUs, uint16_t analysisHz);
    void stepResponseTestReset(void);
    uint16_t stepResponseTestGetWriteIndex(void);
    uint16_t stepResponseTestGetAnalysisHz(void);
    uint16_t stepResponseTestGetSampleCount(void);
    uint16_t stepResponseTestGetResponseCount(void);
    uint8_t stepResponseTestGetActiveBuffer(void);
    bool stepResponseTestJobBusy(void);
    float stepResponseTestGetSample(uint8_t bufferIndex, uint8_t axisIndex, uint8_t signalIndex, uint16_t sampleIndex);
    void stepResponseTestSetAxisMetrics(uint8_t axisIndex, int16_t riseTime01Ms, int16_t overshootPercent, int16_t settlingTime01Ms, int16_t qualityPercent);
    void stepResponseTestSetAxisQualityOnly(uint8_t axisIndex, int16_t qualityPercent);
}

#include "unittest_macros.h"
#include "gtest/gtest.h"

extern "C" {
    int16_t debug[DEBUG16_VALUE_COUNT];
    uint8_t debugMode;
    uint8_t armingFlags;
}

enum {
    STEP_RESPONSE_TEST_ROLL = 0,
    STEP_RESPONSE_TEST_PITCH = 1,
    STEP_RESPONSE_TEST_SETPOINT = 0,
    STEP_RESPONSE_TEST_GYRO = 1,
};

namespace {

// Second-order test plant, bilinear discretisation of
// H(s) = wn^2 / (s^2 + 2*zeta*wn*s + wn^2) at 1 kHz
class SecondOrderPlant {
public:
    SecondOrderPlant(const float naturalHz, const float zeta)
    {
        const double wn = 2.0 * M_PI * naturalHz;
        const double k = 2.0 * 1000.0; // 2 / T at 1 kHz
        const double a0 = k * k + 2.0 * zeta * wn * k + wn * wn;

        b0_ = wn * wn / a0;
        a1_ = 2.0 * (wn * wn - k * k) / a0;
        a2_ = (k * k - 2.0 * zeta * wn * k + wn * wn) / a0;
        reset();
    }

    void reset()
    {
        u1_ = u2_ = y1_ = y2_ = 0.0;
    }

    float process(const float input)
    {
        const double u0 = input;
        const double y0 = b0_ * (u0 + 2.0 * u1_ + u2_) - a1_ * y1_ - a2_ * y2_;

        u2_ = u1_;
        u1_ = u0;
        y2_ = y1_;
        y1_ = y0;

        return (float)y0;
    }

private:
    double b0_;
    double a1_;
    double a2_;
    double u1_, u2_, y1_, y2_;
};

float interpolateCrossing(const float *curve, const int count, const float level)
{
    for (int i = 1; i < count; i++) {
        if (curve[i - 1] < level && curve[i] >= level) {
            const float denominator = curve[i] - curve[i - 1];
            if (fabsf(denominator) < 1.0e-6f) {
                return (float)i;
            }
            return (i - 1) + (level - curve[i - 1]) / denominator;
        }
    }

    return -1.0f;
}

} // namespace

class StepResponseTest : public ::testing::Test {
protected:
    void SetUp() override
    {
        stepResponseTestReset();
        stepResponseConfig_t *config = stepResponseConfigMutable();
        *config = pgResetTemplate_stepResponseConfig;
        config->armedOnly = 0;
        config->expectedHz = 1000;
        config->windowMs = 512;
        config->responseMs = 250;

        memset(debug, 0, sizeof(debug));
        debugMode = DEBUG_STEP_RESPONSE;
        armingFlags = 0;
    }

    static void captureLoop(const timeUs_t currentTimeUs, const int loopIndex)
    {
        stepResponsePidLoopStart(currentTimeUs);
        stepResponseCapture(FD_ROLL, 1000.0f + loopIndex, 2000.0f + loopIndex);
        stepResponseCapture(FD_PITCH, 3000.0f + loopIndex, 4000.0f + loopIndex);
        stepResponseCapture(FD_YAW, -1.0f, -1.0f);
    }

    static void captureLoopPitchFirst(const timeUs_t currentTimeUs, const int loopIndex)
    {
        stepResponsePidLoopStart(currentTimeUs);
        stepResponseCapture(FD_PITCH, 3000.0f + loopIndex, 4000.0f + loopIndex);
        stepResponseCapture(FD_ROLL, 1000.0f + loopIndex, 2000.0f + loopIndex);
    }

    static void captureLoopValues(const timeUs_t currentTimeUs, const float setpoint, const float gyroRate)
    {
        stepResponsePidLoopStart(currentTimeUs);
        stepResponseCapture(FD_ROLL, setpoint, gyroRate);
        stepResponseCapture(FD_PITCH, setpoint, gyroRate);
    }

    static int runAnalysisToCompletion()
    {
        int iterations = 0;

        while (stepResponseUpdateCheck(0, 0) && iterations < 20000) {
            stepResponseUpdate(0);
            iterations++;
        }

        return iterations;
    }
};

TEST_F(StepResponseTest, AccumulatorEmitsAtTargetRate)
{
    uint32_t accumulator = 0;

    for (int i = 0; i < 7; i++) {
        EXPECT_EQ(0, stepResponseUpdateSampleAccumulator(&accumulator, 125, 1000));
    }

    EXPECT_EQ(875000u, accumulator);
    EXPECT_EQ(1, stepResponseUpdateSampleAccumulator(&accumulator, 125, 1000));
    EXPECT_EQ(0u, accumulator);
}

TEST_F(StepResponseTest, AccumulatorKeepsFractionalPidTiming)
{
    uint32_t accumulator = 0;

    EXPECT_EQ(0, stepResponseUpdateSampleAccumulator(&accumulator, 333, 1000));
    EXPECT_EQ(0, stepResponseUpdateSampleAccumulator(&accumulator, 333, 1000));
    EXPECT_EQ(0, stepResponseUpdateSampleAccumulator(&accumulator, 333, 1000));
    EXPECT_EQ(999000u, accumulator);

    EXPECT_EQ(1, stepResponseUpdateSampleAccumulator(&accumulator, 333, 1000));
    EXPECT_EQ(332000u, accumulator);
}

TEST_F(StepResponseTest, SamplesOneKilohertzFromEightKilohertzPidLoop)
{
    for (int loopIndex = 0; loopIndex <= 16; loopIndex++) {
        captureLoop(125 * loopIndex, loopIndex);
    }

    EXPECT_EQ(1000, stepResponseTestGetAnalysisHz());
    EXPECT_EQ(512, stepResponseTestGetSampleCount());
    EXPECT_EQ(2, stepResponseTestGetWriteIndex());

    EXPECT_FLOAT_EQ(1008.0f, stepResponseTestGetSample(0, STEP_RESPONSE_TEST_ROLL, STEP_RESPONSE_TEST_SETPOINT, 0));
    EXPECT_FLOAT_EQ(2008.0f, stepResponseTestGetSample(0, STEP_RESPONSE_TEST_ROLL, STEP_RESPONSE_TEST_GYRO, 0));
    EXPECT_FLOAT_EQ(3008.0f, stepResponseTestGetSample(0, STEP_RESPONSE_TEST_PITCH, STEP_RESPONSE_TEST_SETPOINT, 0));
    EXPECT_FLOAT_EQ(4008.0f, stepResponseTestGetSample(0, STEP_RESPONSE_TEST_PITCH, STEP_RESPONSE_TEST_GYRO, 0));

    EXPECT_FLOAT_EQ(1016.0f, stepResponseTestGetSample(0, STEP_RESPONSE_TEST_ROLL, STEP_RESPONSE_TEST_SETPOINT, 1));
    EXPECT_FLOAT_EQ(3016.0f, stepResponseTestGetSample(0, STEP_RESPONSE_TEST_PITCH, STEP_RESPONSE_TEST_SETPOINT, 1));
}

TEST_F(StepResponseTest, SamplesFractionalPidLoopCadence)
{
    for (int loopIndex = 0; loopIndex <= 10; loopIndex++) {
        captureLoop(333 * loopIndex, loopIndex);
    }

    EXPECT_EQ(3, stepResponseTestGetWriteIndex());
    EXPECT_FLOAT_EQ(1004.0f, stepResponseTestGetSample(0, STEP_RESPONSE_TEST_ROLL, STEP_RESPONSE_TEST_SETPOINT, 0));
    EXPECT_FLOAT_EQ(3004.0f, stepResponseTestGetSample(0, STEP_RESPONSE_TEST_PITCH, STEP_RESPONSE_TEST_SETPOINT, 0));
    EXPECT_FLOAT_EQ(1007.0f, stepResponseTestGetSample(0, STEP_RESPONSE_TEST_ROLL, STEP_RESPONSE_TEST_SETPOINT, 1));
    EXPECT_FLOAT_EQ(3007.0f, stepResponseTestGetSample(0, STEP_RESPONSE_TEST_PITCH, STEP_RESPONSE_TEST_SETPOINT, 1));
    EXPECT_FLOAT_EQ(1010.0f, stepResponseTestGetSample(0, STEP_RESPONSE_TEST_ROLL, STEP_RESPONSE_TEST_SETPOINT, 2));
    EXPECT_FLOAT_EQ(3010.0f, stepResponseTestGetSample(0, STEP_RESPONSE_TEST_PITCH, STEP_RESPONSE_TEST_SETPOINT, 2));
}

TEST_F(StepResponseTest, SamplesAfterBothAxesWithoutPitchSpecificGate)
{
    for (int loopIndex = 0; loopIndex <= 8; loopIndex++) {
        captureLoopPitchFirst(125 * loopIndex, loopIndex);
    }

    EXPECT_EQ(1, stepResponseTestGetWriteIndex());
    EXPECT_FLOAT_EQ(1008.0f, stepResponseTestGetSample(0, STEP_RESPONSE_TEST_ROLL, STEP_RESPONSE_TEST_SETPOINT, 0));
    EXPECT_FLOAT_EQ(3008.0f, stepResponseTestGetSample(0, STEP_RESPONSE_TEST_PITCH, STEP_RESPONSE_TEST_SETPOINT, 0));
}

TEST_F(StepResponseTest, WindowSmallerThanFftSegmentDisablesCapture)
{
    stepResponseConfigMutable()->windowMs = 100;

    for (int loopIndex = 0; loopIndex <= 16; loopIndex++) {
        captureLoop(125 * loopIndex, loopIndex);
    }

    EXPECT_EQ(0, stepResponseTestGetWriteIndex());
    EXPECT_FALSE(stepResponseUpdateCheck(0, 0));
}

TEST_F(StepResponseTest, QualityOnlyUpdateDoesNotOverwriteExistingMetrics)
{
    stepResponseTestSetAxisMetrics(STEP_RESPONSE_TEST_ROLL, 123, 3, 450, 80);
    stepResponseTestSetAxisQualityOnly(STEP_RESPONSE_TEST_ROLL, 5);

    EXPECT_EQ(123, debug[0]);
    EXPECT_EQ(3, debug[1]);
    EXPECT_EQ(450, debug[2]);
    EXPECT_EQ(5, debug[3]);
}

TEST_F(StepResponseTest, QualityOnlyUpdatePublishesNeutralMetricsBeforeFirstValidResult)
{
    stepResponseTestSetAxisQualityOnly(STEP_RESPONSE_TEST_PITCH, 7);

    EXPECT_EQ(0, debug[4]);
    EXPECT_EQ(0, debug[5]);
    EXPECT_EQ(0, debug[6]);
    EXPECT_EQ(7, debug[7]);
}

TEST_F(StepResponseTest, UpdateCheckSignalsActiveWorkOnlyWhenJobPending)
{
    EXPECT_FALSE(stepResponseUpdateCheck(0, 100000));

    // 512 samples at 1 kHz from an 8 kHz loop need 512 * 8 loops plus the
    // initial timing loop
    for (int loopIndex = 0; loopIndex <= 4105; loopIndex++) {
        captureLoop(125 * loopIndex, loopIndex);
    }

    EXPECT_TRUE(stepResponseUpdateCheck(100000, 1));

    debugMode = DEBUG_NONE;
    EXPECT_FALSE(stepResponseUpdateCheck(100000, 1));
}

TEST_F(StepResponseTest, BusyAnalysisDropsNewWindowWithoutStompingMetrics)
{
    // Fill the first window to queue a job
    int loopIndex = 0;
    for (; loopIndex <= 512; loopIndex++) {
        captureLoopValues(1000 * loopIndex, (loopIndex & 64) ? 100.0f : -100.0f, 0.0f);
    }

    EXPECT_TRUE(stepResponseUpdateCheck(0, 0));

    // Start the job but leave it unfinished
    stepResponseUpdate(0);
    EXPECT_TRUE(stepResponseTestJobBusy());

    // Simulate previously published metrics
    stepResponseTestSetAxisMetrics(STEP_RESPONSE_TEST_ROLL, 123, 4, 450, 90);
    stepResponseTestSetAxisMetrics(STEP_RESPONSE_TEST_PITCH, 150, 6, 500, 85);

    // Capture a complete second window while the job is still busy
    for (; loopIndex <= 1024; loopIndex++) {
        captureLoopValues(1000 * loopIndex, (loopIndex & 64) ? 100.0f : -100.0f, 0.0f);
    }

    // The dropped window must not disturb the published metrics
    EXPECT_EQ(123, debug[0]);
    EXPECT_EQ(4, debug[1]);
    EXPECT_EQ(450, debug[2]);
    EXPECT_EQ(90, debug[3]);
    EXPECT_EQ(150, debug[4]);
    EXPECT_EQ(6, debug[5]);
    EXPECT_EQ(500, debug[6]);
    EXPECT_EQ(85, debug[7]);

    // And the write index restarts for the next window
    EXPECT_EQ(0, stepResponseTestGetWriteIndex());
}

TEST_F(StepResponseTest, WeakInputPublishesZeroQualityWithoutMetrics)
{
    stepResponseConfigMutable()->minInput = 100;

    // Setpoint range of 20 deg/s is below the 100 deg/s input gate
    for (int loopIndex = 0; loopIndex <= 513; loopIndex++) {
        const float setpoint = (loopIndex & 32) ? 10.0f : -10.0f;
        captureLoopValues(1000 * loopIndex, setpoint, setpoint * 0.5f);
    }

    EXPECT_TRUE(stepResponseUpdateCheck(0, 0));
    runAnalysisToCompletion();

    for (int i = 0; i < 8; i++) {
        EXPECT_EQ(0, debug[i]);
    }
}

TEST_F(StepResponseTest, EndToEndSecondOrderSystemMetricsInTenthMilliseconds)
{
    stepResponseConfigMutable()->windowMs = 2000;

    const float naturalHz = 30.0f;
    const float zeta = 0.6f;
    SecondOrderPlant plant(naturalHz, zeta);

    // Broadband pseudo-random excitation (sign held for 4 samples) keeps
    // every analysis bin well above the int16 quantisation floor so the
    // coherence estimate stays representative
    uint32_t lcgState = 0x12345678u;
    float setpoint = 300.0f;

    int loopIndex = 0;
    for (; loopIndex <= 2001; loopIndex++) {
        if ((loopIndex & 3) == 0) {
            lcgState = lcgState * 1664525u + 1013904223u;
            setpoint = ((lcgState >> 16) & 1u) ? 300.0f : -300.0f;
        }
        const float gyro = plant.process(setpoint);
        captureLoopValues(1000 * loopIndex, setpoint, gyro);
    }

    EXPECT_TRUE(stepResponseUpdateCheck(0, 0));
    const int iterations = runAnalysisToCompletion();
    EXPECT_LT(iterations, 20000);

    // Reference metrics from the plant's true step response
    const int responseCount = stepResponseTestGetResponseCount();
    ASSERT_GT(responseCount, 0);

    plant.reset();
    float expectedCurve[512];
    for (int i = 0; i < responseCount; i++) {
        expectedCurve[i] = plant.process(1.0f);
    }

    const float riseStart = interpolateCrossing(expectedCurve, responseCount, 0.1f);
    const float riseEnd = interpolateCrossing(expectedCurve, responseCount, 0.9f);
    ASSERT_GE(riseStart, 0.0f);
    ASSERT_GE(riseEnd, riseStart);

    const int expectedRise01Ms = lrintf((riseEnd - riseStart) * 10.0f);
    float expectedPeak = 0.0f;
    for (int i = 0; i < responseCount; i++) {
        expectedPeak = fmaxf(expectedPeak, expectedCurve[i]);
    }
    const int expectedOvershoot = lrintf(fmaxf(0.0f, expectedPeak - 1.0f) * 100.0f);

    for (int axis = 0; axis < 2; axis++) {
        const int base = axis * 4;

        // Rise time in 0.1 ms units within 2.5 ms of the reference
        EXPECT_NEAR(expectedRise01Ms, debug[base + 0], 25);
        // Overshoot within 6 percentage points
        EXPECT_NEAR(expectedOvershoot, debug[base + 1], 6);
        // Settling time valid and below 30 ms for this well-damped plant
        EXPECT_GE(debug[base + 2], 0);
        EXPECT_LE(debug[base + 2], 300);
        // Noiseless LTI system should show high coherence quality
        EXPECT_GE(debug[base + 3], 60);
    }
}
