#ifndef APP_CONTROLLERS_IMU_CONTROLLER_H
#define APP_CONTROLLERS_IMU_CONTROLLER_H

#include "app/controllers/mpu_controller.h"
#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  float q0;
  float q1;
  float q2;
  float q3;
  float integral_x;
  float integral_y;
  float integral_z;
  float accel_x_mps2;
  float accel_y_mps2;
  float accel_z_mps2;
  float gyro_x_rads;
  float gyro_y_rads;
  float gyro_z_rads;
  float roll_deg;
  float pitch_deg;
  float yaw_deg;
  float primary_angle_deg;
  float secondary_angle_deg;
  float compensated_angle_deg;
  float primary_delta_deg;
  float primary_rate_dps;
  float last_primary_angle_deg;
  float last_dt_s;
  uint32_t last_update_ms;
  uint32_t update_count;
  uint16_t stable_sample_count;
  uint8_t stable_cycle_count;
  uint8_t sleep_request;
  uint8_t initialized;
} ImuControllerState;

typedef struct
{
  MpuController *mpu;
  ImuControllerState state;
  float offset_primary_deg;
  float offset_secondary_deg;
  float offset_compensated_deg;
  float proportional_gain;
  float integral_gain;
} ImuController;

void ImuController_Init(ImuController *controller, MpuController *mpu);
void ImuController_ResetFilter(ImuController *controller);
HAL_StatusTypeDef ImuController_Update(ImuController *controller);
uint8_t ImuController_ConsumeSleepRequest(ImuController *controller);
const ImuControllerState *ImuController_GetState(const ImuController *controller);

#ifdef __cplusplus
}
#endif

#endif
