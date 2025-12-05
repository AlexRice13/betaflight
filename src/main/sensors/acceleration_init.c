/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "platform.h"

#ifdef USE_ACC

#include "build/debug.h"

#include "common/utils.h"

#include "config/config_reset.h"
#include "config/feature.h"

#include "drivers/accgyro/accgyro.h"
#include "drivers/accgyro/accgyro_mpu.h"
#include "drivers/accgyro/accgyro_virtual.h"

#include "drivers/accgyro/accgyro_mpu6050.h"
#include "drivers/accgyro/accgyro_mpu6500.h"

#include "drivers/accgyro/accgyro_spi_bmi160.h"
#include "drivers/accgyro/accgyro_spi_bmi270.h"

#include "drivers/accgyro/accgyro_spi_icm20649.h"
#include "drivers/accgyro/accgyro_spi_icm20689.h"
#include "drivers/accgyro/accgyro_spi_icm426xx.h"
#include "drivers/accgyro/accgyro_spi_icm456xx.h"
#include "drivers/accgyro/accgyro_spi_icm40609.h"

#include "drivers/accgyro/accgyro_spi_lsm6dso.h"
#include "drivers/accgyro/accgyro_spi_lsm6dsv16x.h"

#include "drivers/accgyro/accgyro_spi_mpu6000.h"
#include "drivers/accgyro/accgyro_spi_mpu6500.h"
#include "drivers/accgyro/accgyro_spi_mpu9250.h"

#include "config/config.h"

#include "drivers/bus_spi.h"
#include "drivers/time.h"

#include "fc/runtime_config.h"

#include "io/beeper.h"

#include "pg/gyrodev.h"
#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "sensors/boardalignment.h"
#include "sensors/gyro.h"
#include "sensors/gyro_init.h"

#include "acceleration_init.h"

#define CALIBRATING_ACC_CYCLES              400

FAST_DATA_ZERO_INIT accelerationRuntime_t accelerationRuntime;

void resetRollAndPitchTrims(rollAndPitchTrims_t *rollAndPitchTrims)
{
    RESET_CONFIG_2(rollAndPitchTrims_t, rollAndPitchTrims,
        .values.roll = 0,
        .values.pitch = 0,
    );
}

static void setConfigCalibrationCompleted(void)
{
    accelerometerConfigMutable()->accZero.values.calibrationCompleted = 1;
}

bool accHasBeenCalibrated(void)
{
#ifdef SIMULATOR_BUILD
    return true;
#else
    return accelerometerConfig()->accZero.values.calibrationCompleted;
#endif
}

LOCAL_UNUSED_FUNCTION static void accResetRollAndPitchTrims(void)
{
    resetRollAndPitchTrims(&accelerometerConfigMutable()->accelerometerTrims);
}

static void resetFlightDynamicsTrims(flightDynamicsTrims_t *accZero)
{
    accZero->values.roll = 0;
    accZero->values.pitch = 0;
    accZero->values.yaw = 0;
    accZero->values.calibrationCompleted = 0;
}

static void pgResetFn_accelerometerConfig(accelerometerConfig_t *instance)
{
    RESET_CONFIG_2(accelerometerConfig_t, instance,
        .acc_lpf_hz = 25, // ATTITUDE/IMU runs at 100Hz (acro) or 500Hz (level modes) so we need to set 50 Hz (or lower) to avoid aliasing
        .acc_hardware = ACC_DEFAULT,
        .acc_high_fsr = false,
        .inflight_acc_cal_window = 50, // default window length preserves existing behavior
    );
    resetRollAndPitchTrims(&instance->accelerometerTrims);
    resetFlightDynamicsTrims(&instance->accZero);
}

PG_REGISTER_WITH_RESET_FN(accelerometerConfig_t, accelerometerConfig, PG_ACCELEROMETER_CONFIG, 3);

extern bool AccInflightCalibrationMeasurementDone;
extern bool AccInflightCalibrationSavetoEEProm;
extern bool AccInflightCalibrationActive;

#define INFLIGHT_ACC_CAL_WINDOW_MIN 10
#define INFLIGHT_ACC_CAL_WINDOW_MAX 200
#define INFLIGHT_ACC_CAL_DEBOUNCE_MS 500

static uint8_t getInflightAccCalWindow(void)
{
    uint8_t window = accelerometerConfig()->inflight_acc_cal_window;
    // Clamp to valid range
    if (window < INFLIGHT_ACC_CAL_WINDOW_MIN) {
        window = INFLIGHT_ACC_CAL_WINDOW_MIN;
    } else if (window > INFLIGHT_ACC_CAL_WINDOW_MAX) {
        window = INFLIGHT_ACC_CAL_WINDOW_MAX;
    }
    return window;
}

static bool accDetect(accDev_t *dev, accelerationSensor_e accHardwareToUse)
{
    accelerationSensor_e accHardware = ACC_NONE;

retry:

    switch (accHardwareToUse) {
    case ACC_DEFAULT:
        FALLTHROUGH;

#ifdef USE_ACC_MPU6050
    case ACC_MPU6050: // MPU6050
        if (mpu6050AccDetect(dev)) {
            accHardware = ACC_MPU6050;
            break;
        }
        FALLTHROUGH;
#endif

#ifdef USE_ACC_SPI_MPU6000
    case ACC_MPU6000:
        if (mpu6000SpiAccDetect(dev)) {
            accHardware = ACC_MPU6000;
            break;
        }
        FALLTHROUGH;
#endif

#ifdef USE_ACC_SPI_MPU9250
    case ACC_MPU9250:
        if (mpu9250SpiAccDetect(dev)) {
            accHardware = ACC_MPU9250;
            break;
        }
        FALLTHROUGH;
#endif

#if defined(USE_ACC_MPU6500) || defined(USE_ACC_SPI_MPU6500)
    case ACC_MPU6500:
    case ACC_ICM20601:
    case ACC_ICM20602:
    case ACC_ICM20608G:
#ifdef USE_ACC_SPI_MPU6500
        if (mpu6500SpiAccDetect(dev)) {
#else
        if (mpu6500AccDetect(dev)) {
#endif
            switch (dev->mpuDetectionResult.sensor) {
            case MPU_9250_SPI:
                accHardware = ACC_MPU9250;
                break;
            case ICM_20601_SPI:
                accHardware = ACC_ICM20601;
                break;
            case ICM_20602_SPI:
                accHardware = ACC_ICM20602;
                break;
            case ICM_20608_SPI:
                accHardware = ACC_ICM20608G;
                break;
            default:
                accHardware = ACC_MPU6500;
            }
            break;
        }
        FALLTHROUGH;
#endif

#ifdef USE_ACC_SPI_ICM20649
    case ACC_ICM20649:
        if (icm20649SpiAccDetect(dev)) {
            accHardware = ACC_ICM20649;
            break;
        }
        FALLTHROUGH;
#endif

#ifdef USE_ACC_SPI_ICM20689
    case ACC_ICM20689:
        if (icm20689SpiAccDetect(dev)) {
            accHardware = ACC_ICM20689;
            break;
        }
        FALLTHROUGH;
#endif

#if defined(USE_ACC_SPI_ICM42605) || defined(USE_ACC_SPI_ICM42688P) || defined(USE_ACCGYRO_IIM42652) || defined(USE_ACCGYRO_IIM42653)
    case ACC_ICM42605:
    case ACC_ICM42688P:
    case ACC_IIM42652:
    case ACC_IIM42653:
        if (icm426xxSpiAccDetect(dev)) {
            switch (dev->mpuDetectionResult.sensor) {
            case ICM_42605_SPI:
                accHardware = ACC_ICM42605;
                break;
            case ICM_42688P_SPI:
                accHardware = ACC_ICM42688P;
                break;
            case IIM_42652_SPI:
                accHardware = ACC_IIM42652;
                break;
            case IIM_42653_SPI:
                accHardware = ACC_IIM42653;
                break;
            default:
                accHardware = ACC_NONE;
                break;
            }
            break;
        }
        FALLTHROUGH;
#endif

#if defined(USE_ACCGYRO_ICM45686) || defined(USE_ACCGYRO_ICM45605)
    case ACC_ICM45686:
    case ACC_ICM45605:
        if (icm456xxSpiAccDetect(dev)) {
            switch (dev->mpuDetectionResult.sensor) {
            case ICM_45686_SPI:
                accHardware = ACC_ICM45686;
                break;
            case ICM_45605_SPI:
                accHardware = ACC_ICM45605;
                break;
            default:
                accHardware = ACC_NONE;
                break;
            }
            break;
        }
        FALLTHROUGH;
#endif

#ifdef USE_ACCGYRO_BMI160
    case ACC_BMI160:
        if (bmi160SpiAccDetect(dev)) {
            accHardware = ACC_BMI160;
            break;
        }
        FALLTHROUGH;
#endif

#ifdef USE_ACCGYRO_BMI270
    case ACC_BMI270:
        if (bmi270SpiAccDetect(dev)) {
            accHardware = ACC_BMI270;
            break;
        }
        FALLTHROUGH;
#endif

#ifdef USE_ACCGYRO_LSM6DSO
    case ACC_LSM6DSO:
        if (lsm6dsoSpiAccDetect(dev)) {
            accHardware = ACC_LSM6DSO;
            break;
        }
        FALLTHROUGH;
#endif

#ifdef USE_ACCGYRO_LSM6DSV16X
    case ACC_LSM6DSV16X:
        if (lsm6dsv16xSpiAccDetect(dev)) {
            accHardware = ACC_LSM6DSV16X;
            break;
        }
        FALLTHROUGH;
#endif

#ifdef USE_ACCGYRO_ICM40609D
    case ACC_ICM40609D:
        if (icm40609SpiAccDetect(dev)) {
            accHardware = ACC_ICM40609D;
            break;
        }
        FALLTHROUGH;
#endif

#ifdef USE_VIRTUAL_ACC
    case ACC_VIRTUAL:
        if (virtualAccDetect(dev)) {
            accHardware = ACC_VIRTUAL;
            break;
        }
        FALLTHROUGH;
#endif

    default:
    case ACC_NONE: // disable ACC
        UNUSED(dev);
        accHardware = ACC_NONE;
        break;
    }

    // Found anything? Check if error or ACC is really missing.
    if (accHardware == ACC_NONE && accHardwareToUse != ACC_DEFAULT && accHardwareToUse != ACC_NONE) {
        // Nothing was found and we have a forced sensor that isn't present.
        accHardwareToUse = ACC_DEFAULT;
        goto retry;
    }

    if (accHardware == ACC_NONE) {
        return false;
    }

    detectedSensors[SENSOR_INDEX_ACC] = accHardware;
    sensorsSet(SENSOR_ACC);
    return true;
}

void accInitFilters(void)
{
    // Only set the lowpass cutoff if the ACC sample rate is detected otherwise
    // the filter initialization is not defined (sample rate = 0)
    accelerationRuntime.accLpfCutHz = (acc.sampleRateHz) ? accelerometerConfig()->acc_lpf_hz : 0;
    if (accelerationRuntime.accLpfCutHz) {
        const float k = pt2FilterGain(accelerationRuntime.accLpfCutHz, HZ_TO_INTERVAL(acc.sampleRateHz));
        for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
            pt2FilterInit(&accelerationRuntime.accFilter[axis], k);
        }
    }
}

bool accInit(uint16_t accSampleRateHz)
{
    memset(&acc, 0, sizeof(acc));
    // copy over the common gyro mpu settings
    acc.dev.gyro = gyroActiveDev();
    acc.dev.mpuDetectionResult = *gyroMpuDetectionResult();
    acc.dev.acc_high_fsr = accelerometerConfig()->acc_high_fsr;

    // Copy alignment from active gyro, as all production boards use acc-gyro-combi chip.
    // Exception is STM32F411DISCOVERY, and (may be) handled in future enhancement.

    sensor_align_e alignment = gyroDeviceConfig(firstEnabledGyro())->alignment;
    const sensorAlignment_t* customAlignment = &gyroDeviceConfig(firstEnabledGyro())->customAlignment;

    acc.dev.accAlign = alignment;
    buildRotationMatrixFromAngles(&acc.dev.rotationMatrix, customAlignment);

    if (!accDetect(&acc.dev, accelerometerConfig()->acc_hardware)) {
        return false;
    }
    acc.dev.acc_1G = 256; // set default
    acc.dev.initFn(&acc.dev); // driver initialisation
    acc.dev.acc_1G_rec = 1.0f / acc.dev.acc_1G;

    acc.sampleRateHz = accSampleRateHz;
    accInitFilters();

    return true;
}

void accStartCalibration(void)
{
    accelerationRuntime.calibratingA = CALIBRATING_ACC_CYCLES;
}

bool accIsCalibrationComplete(void)
{
    return accelerationRuntime.calibratingA == 0;
}

static bool isOnFinalAccelerationCalibrationCycle(void)
{
    return accelerationRuntime.calibratingA == 1;
}

static bool isOnFirstAccelerationCalibrationCycle(void)
{
    return accelerationRuntime.calibratingA == CALIBRATING_ACC_CYCLES;
}

void performAccelerometerCalibration(rollAndPitchTrims_t *rollAndPitchTrims)
{
    static int32_t a[3];

    for (int axis = 0; axis < 3; axis++) {

        // Reset a[axis] at start of calibration
        if (isOnFirstAccelerationCalibrationCycle()) {
            a[axis] = 0;
        }

        // Sum up CALIBRATING_ACC_CYCLES readings
        a[axis] += acc.accADC.v[axis];

        // Reset global variables to prevent other code from using un-calibrated data
        acc.accADC.v[axis] = 0;
        accelerationRuntime.accelerationTrims->raw[axis] = 0;
    }

    if (isOnFinalAccelerationCalibrationCycle()) {
        // Calculate average, shift Z down by acc_1G and store values in EEPROM at end of calibration
        accelerationRuntime.accelerationTrims->raw[X] = (a[X] + (CALIBRATING_ACC_CYCLES / 2)) / CALIBRATING_ACC_CYCLES;
        accelerationRuntime.accelerationTrims->raw[Y] = (a[Y] + (CALIBRATING_ACC_CYCLES / 2)) / CALIBRATING_ACC_CYCLES;
        accelerationRuntime.accelerationTrims->raw[Z] = (a[Z] + (CALIBRATING_ACC_CYCLES / 2)) / CALIBRATING_ACC_CYCLES - acc.dev.acc_1G;

        resetRollAndPitchTrims(rollAndPitchTrims);
        setConfigCalibrationCompleted();

        saveConfigAndNotify();
    }

    accelerationRuntime.calibratingA--;
}

void performInflightAccelerationCalibration(rollAndPitchTrims_t *rollAndPitchTrims)
{
    UNUSED(rollAndPitchTrims);
    
    static int32_t b[3];
    static uint8_t windowLength = 0;
    static uint16_t sampleCount = 0;
    static bool wasActive = false;
    static uint32_t lastSaveTime = 0;

    // Debounce logic: ignore activation if within debounce period of last save
    bool effectiveActive = AccInflightCalibrationActive;
    if (millis() - lastSaveTime < INFLIGHT_ACC_CAL_DEBOUNCE_MS) {
        effectiveActive = false;
    }

    // When calibration becomes active (AUX switch ON), reset accumulators and re-read window length
    if (effectiveActive && !wasActive) {
        windowLength = getInflightAccCalWindow();
        sampleCount = 0;
        for (int axis = 0; axis < 3; axis++) {
            b[axis] = 0;
        }
    }
    
    // When calibration becomes inactive (AUX switch OFF), trigger EEPROM save if measurement was done
    if (!effectiveActive && wasActive && AccInflightCalibrationMeasurementDone) {
        AccInflightCalibrationSavetoEEProm = true;
        AccInflightCalibrationMeasurementDone = false;
    }

    // Collect samples and apply calibration while BOXCALIB AUX is ON
    if (effectiveActive) {
        // Accumulate samples
        for (int axis = 0; axis < 3; axis++) {
            b[axis] += acc.accADC.v[axis];
        }
        sampleCount++;

        // When we have collected windowLength samples, calculate and apply new calibration
        if (sampleCount >= windowLength) {
            // Calculate average and apply continuously to acc trims
            accelerationRuntime.accelerationTrims->raw[X] = b[X] / windowLength;
            accelerationRuntime.accelerationTrims->raw[Y] = b[Y] / windowLength;
            accelerationRuntime.accelerationTrims->raw[Z] = b[Z] / windowLength - acc.dev.acc_1G;

            // Reset accumulators for next window
            for (int axis = 0; axis < 3; axis++) {
                b[axis] = 0;
            }
            sampleCount = 0;
            
            // Mark that we have at least one calibration measurement
            AccInflightCalibrationMeasurementDone = true;
        }
    } else {
        // Calibration not active (AUX switch OFF), reset sample count and accumulators
        sampleCount = 0;
        for (int axis = 0; axis < 3; axis++) {
            b[axis] = 0;
        }
    }
    
    wasActive = effectiveActive;

    // Save calibration to EEPROM when requested
    if (AccInflightCalibrationSavetoEEProm) {
        AccInflightCalibrationSavetoEEProm = false;
        
        // Current trims are already applied, just save them
        setConfigCalibrationCompleted();

        saveConfigAndNotify();
        lastSaveTime = millis();
    }
}

void setAccelerationTrims(flightDynamicsTrims_t *accelerationTrimsToUse)
{
    accelerationRuntime.accelerationTrims = accelerationTrimsToUse;
}

void applyAccelerometerTrimsDelta(rollAndPitchTrims_t *rollAndPitchTrimsDelta)
{
    accelerometerConfigMutable()->accelerometerTrims.values.roll += rollAndPitchTrimsDelta->values.roll;
    accelerometerConfigMutable()->accelerometerTrims.values.pitch += rollAndPitchTrimsDelta->values.pitch;
}
#endif
