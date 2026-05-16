#include "app/controllers/mpu_controller.h"

#include "wwdg.h"

static void MpuController_RefreshWatchdogDuringConfigure(void)
{
  (void)HAL_WWDG_Refresh(&hwwdg);
}

void MpuController_Init(MpuController *controller, Mpu6050Driver *driver)
{
  if (controller == NULL)
  {
    return;
  }

  controller->driver = driver;
  controller->who_am_i = 0;
  controller->present = 0;
  controller->last_raw.accel_x = 0;
  controller->last_raw.accel_y = 0;
  controller->last_raw.accel_z = 0;
  controller->last_raw.gyro_x = 0;
  controller->last_raw.gyro_y = 0;
  controller->last_raw.gyro_z = 0;
  controller->last_scaled.accel_x_mps2 = 0.0f;
  controller->last_scaled.accel_y_mps2 = 0.0f;
  controller->last_scaled.accel_z_mps2 = 0.0f;
  controller->last_scaled.gyro_x_dps = 0.0f;
  controller->last_scaled.gyro_y_dps = 0.0f;
  controller->last_scaled.gyro_z_dps = 0.0f;
}

HAL_StatusTypeDef MpuController_Probe(MpuController *controller)
{
  if ((controller == NULL) || (controller->driver == NULL))
  {
    return HAL_ERROR;
  }

  HAL_StatusTypeDef status = Mpu6050Driver_ReadWhoAmI(controller->driver, &controller->who_am_i);
  controller->present = (uint8_t)((status == HAL_OK) &&
                                  ((controller->who_am_i == 0x68U) || (controller->who_am_i == 0x69U)));
  return status;
}

HAL_StatusTypeDef MpuController_Configure(MpuController *controller)
{
  if ((controller == NULL) || (controller->driver == NULL) || (controller->present == 0U))
  {
    return HAL_ERROR;
  }

  return Mpu6050Driver_ConfigureWithCallback(controller->driver,
                                             MpuController_RefreshWatchdogDuringConfigure);
}

HAL_StatusTypeDef MpuController_UpdateRaw(MpuController *controller)
{
  if ((controller == NULL) || (controller->driver == NULL) || (controller->present == 0U))
  {
    return HAL_ERROR;
  }

  HAL_StatusTypeDef status = Mpu6050Driver_ReadRawSample(controller->driver, &controller->last_raw);
  if (status == HAL_OK)
  {
    Mpu6050Driver_ScaleRawSample(&controller->last_raw, &controller->last_scaled);
  }

  return status;
}

HAL_StatusTypeDef MpuController_UpdateScaled(MpuController *controller)
{
  if ((controller == NULL) || (controller->driver == NULL) || (controller->present == 0U))
  {
    return HAL_ERROR;
  }

  return Mpu6050Driver_ReadScaledSample(controller->driver, &controller->last_scaled);
}
