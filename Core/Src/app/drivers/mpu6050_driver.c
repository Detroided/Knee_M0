#include "app/drivers/mpu6050_driver.h"

#define MPU6050_ACCEL_SCALE_MPS2 ((float)(9.8f / 8192.0f))
#define MPU6050_GYRO_SCALE_DPS   ((float)0.030517578f)

static void Mpu6050Driver_BusRecovery(Mpu6050Driver *driver)
{
  GPIO_InitTypeDef gpio = {0};

  if ((driver == NULL) || (driver->hi2c == NULL) || (driver->hi2c->Instance != I2C1))
  {
    return;
  }

  __HAL_RCC_I2C1_FORCE_RESET();
  __HAL_RCC_I2C1_RELEASE_RESET();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  gpio.Pin = I2C1_SCL_MPU6050_Pin | I2C1_SDA_MPU6050_Pin;
  gpio.Mode = GPIO_MODE_OUTPUT_OD;
  gpio.Pull = GPIO_PULLUP;
  gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &gpio);

  for (uint32_t i = 0; i < 16U; i++)
  {
    HAL_GPIO_WritePin(I2C1_SCL_MPU6050_GPIO_Port, I2C1_SCL_MPU6050_Pin, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(I2C1_SCL_MPU6050_GPIO_Port, I2C1_SCL_MPU6050_Pin, GPIO_PIN_SET);
    HAL_Delay(1);
  }

  gpio.Mode = GPIO_MODE_AF_OD;
  gpio.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &gpio);

  (void)HAL_I2C_DeInit(driver->hi2c);
  (void)HAL_I2C_Init(driver->hi2c);
}

static HAL_StatusTypeDef Mpu6050Driver_WriteReg(Mpu6050Driver *driver, uint8_t reg, uint8_t value)
{
  if ((driver == NULL) || (driver->hi2c == NULL))
  {
    return HAL_ERROR;
  }

  HAL_StatusTypeDef status = HAL_I2C_Mem_Write(driver->hi2c, MPU6050_I2C_ADDRESS_WRITE, reg,
                                               I2C_MEMADD_SIZE_8BIT, &value, 1, 10);
  if (status != HAL_OK)
  {
    Mpu6050Driver_BusRecovery(driver);
  }

  return status;
}

void Mpu6050Driver_Init(Mpu6050Driver *driver, I2C_HandleTypeDef *hi2c)
{
  if (driver == NULL)
  {
    return;
  }

  driver->hi2c = hi2c;
}

HAL_StatusTypeDef Mpu6050Driver_ReadWhoAmI(Mpu6050Driver *driver, uint8_t *who_am_i)
{
  if ((driver == NULL) || (driver->hi2c == NULL) || (who_am_i == NULL))
  {
    return HAL_ERROR;
  }

  HAL_StatusTypeDef status = HAL_I2C_Mem_Read(driver->hi2c, MPU6050_I2C_ADDRESS_READ,
                                              MPU6050_WHO_AM_I_REG,
                                              I2C_MEMADD_SIZE_8BIT, who_am_i, 1, 10);
  if (status != HAL_OK)
  {
    Mpu6050Driver_BusRecovery(driver);
  }

  return status;
}

HAL_StatusTypeDef Mpu6050Driver_ConfigureWithCallback(Mpu6050Driver *driver,
                                                      Mpu6050DriverStepCallback step_callback)
{
  static const struct
  {
    uint8_t reg;
    uint8_t value;
  } sequence[] = {
    {0x6B, 0x00}, {0x19, 0x04}, {0x1A, 0x06}, {0x38, 0x00}, {0x6A, 0x00},
    {0x23, 0x00}, {0x1C, 0x08}, {0x1B, 0x10}, {0x6C, 0x00}, {0x1F, 0x06},
    {0x20, 0x0A}, {0x1C, 0x1C}, {0x1A, 0x06}, {0x37, 0x10}, {0x38, 0x40},
  };

  for (uint32_t i = 0; i < (sizeof(sequence) / sizeof(sequence[0])); i++)
  {
    HAL_StatusTypeDef status = Mpu6050Driver_WriteReg(driver, sequence[i].reg, sequence[i].value);
    if (status != HAL_OK)
    {
      return status;
    }
    HAL_Delay(20);
    if (step_callback != NULL)
    {
      step_callback();
    }
  }

  return HAL_OK;
}

HAL_StatusTypeDef Mpu6050Driver_Configure(Mpu6050Driver *driver)
{
  return Mpu6050Driver_ConfigureWithCallback(driver, NULL);
}

void Mpu6050Driver_ScaleRawSample(const Mpu6050RawSample *raw, Mpu6050ScaledSample *scaled)
{
  if ((raw == NULL) || (scaled == NULL))
  {
    return;
  }

  scaled->accel_x_mps2 = (float)raw->accel_x * MPU6050_ACCEL_SCALE_MPS2;
  scaled->accel_y_mps2 = (float)raw->accel_y * MPU6050_ACCEL_SCALE_MPS2;
  scaled->accel_z_mps2 = (float)raw->accel_z * MPU6050_ACCEL_SCALE_MPS2;
  scaled->gyro_x_dps = (float)raw->gyro_x * MPU6050_GYRO_SCALE_DPS;
  scaled->gyro_y_dps = (float)raw->gyro_y * MPU6050_GYRO_SCALE_DPS;
  scaled->gyro_z_dps = (float)raw->gyro_z * MPU6050_GYRO_SCALE_DPS;
}

HAL_StatusTypeDef Mpu6050Driver_ReadScaledSample(Mpu6050Driver *driver,
                                                 Mpu6050ScaledSample *sample)
{
  Mpu6050RawSample raw;
  HAL_StatusTypeDef status;

  if (sample == NULL)
  {
    return HAL_ERROR;
  }

  status = Mpu6050Driver_ReadRawSample(driver, &raw);
  if (status != HAL_OK)
  {
    Mpu6050Driver_BusRecovery(driver);
    return status;
  }

  Mpu6050Driver_ScaleRawSample(&raw, sample);
  return HAL_OK;
}

HAL_StatusTypeDef Mpu6050Driver_ReadInt16(Mpu6050Driver *driver, uint8_t reg_hi, int16_t *value)
{
  uint8_t bytes[2] = {0, 0};

  if ((driver == NULL) || (driver->hi2c == NULL) || (value == NULL))
  {
    return HAL_ERROR;
  }

  HAL_StatusTypeDef status = HAL_I2C_Mem_Read(driver->hi2c, MPU6050_I2C_ADDRESS_READ, reg_hi,
                                              I2C_MEMADD_SIZE_8BIT, bytes, 2, 10);
  if (status != HAL_OK)
  {
    return status;
  }

  *value = (int16_t)(((uint16_t)bytes[0] << 8) | bytes[1]);
  return HAL_OK;
}

HAL_StatusTypeDef Mpu6050Driver_ReadRawSample(Mpu6050Driver *driver, Mpu6050RawSample *sample)
{
  if ((driver == NULL) || (sample == NULL))
  {
    return HAL_ERROR;
  }

  HAL_StatusTypeDef status = Mpu6050Driver_ReadInt16(driver, 0x3B, &sample->accel_x);
  if (status != HAL_OK)
  {
    return status;
  }

  status = Mpu6050Driver_ReadInt16(driver, 0x3D, &sample->accel_y);
  if (status != HAL_OK)
  {
    return status;
  }

  status = Mpu6050Driver_ReadInt16(driver, 0x3F, &sample->accel_z);
  if (status != HAL_OK)
  {
    return status;
  }

  status = Mpu6050Driver_ReadInt16(driver, 0x43, &sample->gyro_x);
  if (status != HAL_OK)
  {
    return status;
  }

  status = Mpu6050Driver_ReadInt16(driver, 0x45, &sample->gyro_y);
  if (status != HAL_OK)
  {
    return status;
  }

  status = Mpu6050Driver_ReadInt16(driver, 0x47, &sample->gyro_z);
  if (status != HAL_OK)
  {
    return status;
  }

  return HAL_OK;
}
