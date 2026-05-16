#ifndef APP_DRIVERS_MPU6050_DRIVER_H
#define APP_DRIVERS_MPU6050_DRIVER_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MPU6050_I2C_ADDRESS_READ  ((uint16_t)0xD1)
#define MPU6050_I2C_ADDRESS_WRITE ((uint16_t)0xD0)
#define MPU6050_WHO_AM_I_REG      ((uint8_t)0x75)

typedef struct
{
  I2C_HandleTypeDef *hi2c;
} Mpu6050Driver;

typedef struct
{
  int16_t accel_x;
  int16_t accel_y;
  int16_t accel_z;
  int16_t gyro_x;
  int16_t gyro_y;
  int16_t gyro_z;
} Mpu6050RawSample;

typedef struct
{
  float accel_x_mps2;
  float accel_y_mps2;
  float accel_z_mps2;
  float gyro_x_dps;
  float gyro_y_dps;
  float gyro_z_dps;
} Mpu6050ScaledSample;

typedef void (*Mpu6050DriverStepCallback)(void);

void Mpu6050Driver_Init(Mpu6050Driver *driver, I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef Mpu6050Driver_ReadWhoAmI(Mpu6050Driver *driver, uint8_t *who_am_i);
HAL_StatusTypeDef Mpu6050Driver_Configure(Mpu6050Driver *driver);
HAL_StatusTypeDef Mpu6050Driver_ConfigureWithCallback(Mpu6050Driver *driver,
                                                      Mpu6050DriverStepCallback step_callback);
HAL_StatusTypeDef Mpu6050Driver_ReadRawSample(Mpu6050Driver *driver, Mpu6050RawSample *sample);
void Mpu6050Driver_ScaleRawSample(const Mpu6050RawSample *raw, Mpu6050ScaledSample *scaled);
HAL_StatusTypeDef Mpu6050Driver_ReadScaledSample(Mpu6050Driver *driver,
                                                 Mpu6050ScaledSample *sample);
HAL_StatusTypeDef Mpu6050Driver_ReadInt16(Mpu6050Driver *driver, uint8_t reg_hi, int16_t *value);

#ifdef __cplusplus
}
#endif

#endif
