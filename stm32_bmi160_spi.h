#ifndef STM32_BMI160_SPI_H_
#define STM32_BMI160_SPI_H_

#include "stm32f4xx_hal.h"
#include "main.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "stm32_bmi160_defines.h"

HAL_StatusTypeDef BMI160_SPI_Initialize(void);
HAL_StatusTypeDef BMI160_SPI_PerformSoftReset(void);
uint8_t BMI160_SPI_ReadChipID(void);
HAL_StatusTypeDef BMI160_SPI_PowerUpAccelerometer(void);
HAL_StatusTypeDef BMI160_SPI_PowerUpGyroscope(void);
uint8_t BMI160_SPI_CheckSensorID(void);
HAL_StatusTypeDef BMI160_SPI_ReadGyro(int16_t* x, int16_t* y, int16_t* z);
float BMI160_SPI_ScaledData(const int16_t gRaw, const BMI160GyroRange gyroRange);
int16_t BMI160_SPI_GetTemperature(void);
float BMI160_SPI_ConvertRawTemp(int16_t tempRaw);
int16_t BMI160_SPI_GetAccelerationX(void);
int16_t BMI160_SPI_GetAccelerationY(void);
int16_t BMI160_SPI_GetAccelerationZ(void);
HAL_StatusTypeDef BMI160_GetAcceleration(int16_t* x, int16_t* y, int16_t* z);
HAL_StatusTypeDef BMI160_GetMotion6(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz);
BMI160GyroRate BMI160_SPI_GetGyroRate(void);
HAL_StatusTypeDef BMI160_SPI_SetGyroRate(const BMI160GyroRate rate);
BMI160AccelRate BMI160_SPI_GetAccelRate(void);
HAL_StatusTypeDef BMI160_SPI_SetAccelRate(const BMI160AccelRate rate);
BMI160DLPFMode BMI160_SPI_GetGyroDLPFMode(void);
HAL_StatusTypeDef BMI160_SPI_SetGyroDLPFMode(BMI160DLPFMode mode);
BMI160DLPFMode BMI160_SPI_GetAccelDLPFMode(void);
HAL_StatusTypeDef BMI160_SPI_SetAccelDLPFMode(BMI160DLPFMode mode);
BMI160GyroRange BMI160_SPI_GetFullScaleGyroRange(void);
HAL_StatusTypeDef BMI160_SPI_SetFullScaleGyroRange(const BMI160GyroRange range);
BMI160AccelRange BMI160_SPI_GetFullScaleAccelRange(void);
HAL_StatusTypeDef BMI160_SPI_SetFullScaleAccelRange(const BMI160AccelRange range);
uint8_t BMI160_SPI_GetAccelOffsetEnabled(void);
HAL_StatusTypeDef BMI160_SPI_SetAccelOffsetEnabled(const uint8_t enabled);
HAL_StatusTypeDef BMI160_SPI_AutoCalibrateXAccelOffset(int target);
HAL_StatusTypeDef BMI160_SPI_AutoCalibrateYAccelOffset(int target);
HAL_StatusTypeDef BMI160_SPI_AutoCalibrateZAccelOffset(int target);
int8_t BMI160_SPI_GetXAccelOffset(void);
HAL_StatusTypeDef BMI160_SPI_SetXAccelOffset(int8_t offset);
int8_t BMI160_SPI_GetYAccelOffset(void);
HAL_StatusTypeDef BMI160_SPI_SetYAccelOffset(int8_t offset);
int8_t BMI160_SPI_GetZAccelOffset(void);
HAL_StatusTypeDef BMI160_SPI_SetZAccelOffset(int8_t offset);
uint8_t BMI160_SPI_GetGyroOffsetEnabled(void);
HAL_StatusTypeDef BMI160_SPI_SetGyroOffsetEnabled(uint8_t enabled);
HAL_StatusTypeDef BMI160_SPI_AutoCalibrateGyroOffset(void);
HAL_StatusTypeDef BMI160_SPI_SetXGyroOffset(int16_t offset);
int16_t BMI160_SPI_GetYGyroOffset(void);
int16_t BMI160_SPI_GetZGyroOffset(void);
HAL_StatusTypeDef BMI160_SPI_SetZGyroOffset(int16_t offset) ;
uint8_t BMI160_SPI_GetFreefallDetectionThreshold(void);
HAL_StatusTypeDef BMI160_SPI_SetFreefallDetectionThreshold(uint8_t threshold);
uint8_t BMI160_SPI_GetFreefallDetectionDuration(void);
HAL_StatusTypeDef BMI160_SPI_SetFreefallDetectionDuration(uint8_t duration);
uint8_t BMI160_SPI_GetShockDetectionThreshold(void);
HAL_StatusTypeDef BMI160_SPI_SetShockDetectionThreshold(uint8_t threshold);
uint8_t BMI160_SPI_GetShockDetectionDuration(void);
HAL_StatusTypeDef BMI160_SPI_SetShockDetectionDuration(uint8_t duration);
uint8_t BMI160_SPI_GetStepDetectionMode(void);
HAL_StatusTypeDef BMI160_SPI_SetStepDetectionMode(BMI160StepMode mode);
uint8_t BMI160_SPI_GetStepCountEnabled(void);
HAL_StatusTypeDef BMI160_SPI_SetStepCountEnabled(uint8_t enabled);
uint16_t BMI160_SPI_GetStepCount();
HAL_StatusTypeDef BMI160_SPI_ResetStepCount(void);
HAL_StatusTypeDef BMI160_SPI_GetMotionDetectionThreshold(void);
HAL_StatusTypeDef BMI160_SPI_SetMotionDetectionThreshold(uint8_t threshold);
uint8_t BMI160_SPI_GetMotionDetectionDuration(void);
HAL_StatusTypeDef BMI160_SPI_SetMotionDetectionDuration(uint8_t samples);
uint8_t BMI160_SPI_GetZeroMotionDetectionThreshold(void);
HAL_StatusTypeDef BMI160_SPI_SetZeroMotionDetectionThreshold(const uint8_t threshold);
uint8_t BMI160_SPI_GetZeroMotionDetectionDuration(void);
HAL_StatusTypeDef BMI160_SPI_SetZeroMotionDetectionDuration(const uint8_t duration);
uint8_t BMI160_SPI_GetTapDetectionThreshold(void);
HAL_StatusTypeDef BMI160_SPI_SetTapDetectionThreshold(const uint8_t threshold);
uint8_t BMI160_SPI_GetTapShockDuration(void);


#endif /* STM32_BMI160_H_ */
