#ifndef APP_CONTROLLERS_MPU_CONTROLLER_H
#define APP_CONTROLLERS_MPU_CONTROLLER_H

#include "app/drivers/mpu6050_driver.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  Mpu6050Driver *driver;
  uint8_t who_am_i;
  uint8_t present;
  Mpu6050RawSample last_raw;
  Mpu6050ScaledSample last_scaled;
} MpuController;

void MpuController_Init(MpuController *controller, Mpu6050Driver *driver);
HAL_StatusTypeDef MpuController_Probe(MpuController *controller);
HAL_StatusTypeDef MpuController_Configure(MpuController *controller);
HAL_StatusTypeDef MpuController_UpdateRaw(MpuController *controller);
HAL_StatusTypeDef MpuController_UpdateScaled(MpuController *controller);

#ifdef __cplusplus
}
#endif

#endif
