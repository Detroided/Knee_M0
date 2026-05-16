#include "app/controllers/imu_controller.h"

#include <math.h>
#include <string.h>

#define IMU_DEG_TO_RAD              ((float)0.01745329238474369f)
#define IMU_RAD_TO_DEG              ((float)57.29577951308232f)
#define IMU_MIN_DT_S                ((float)0.001f)
#define IMU_MAX_DT_S                ((float)0.020f)
#define IMU_DEFAULT_PRIMARY_OFFSET  ((float)0.0f)
#define IMU_DEFAULT_SECONDARY_OFFSET ((float)5.74f)
#define IMU_DEFAULT_COMP_OFFSET     ((float)0.0f)
#define IMU_DEFAULT_KP              ((float)1.6f)
#define IMU_DEFAULT_KI              ((float)0.001f)
#define IMU_SLEEP_ANGLE_LIMIT_DEG   ((float)60.0f)
#define IMU_SLEEP_DELTA_LIMIT_DEG   ((float)2.0f)
#define IMU_SLEEP_STABLE_SAMPLES    ((uint16_t)1000U)
#define IMU_SLEEP_STABLE_CYCLES     ((uint8_t)11U)

static float ImuController_InvSqrt(float value)
{
  if (value <= 0.0f)
  {
    return 0.0f;
  }

  return 1.0f / sqrtf(value);
}

static float ImuController_ClampDt(uint32_t now_ms, uint32_t previous_ms)
{
  float dt;

  if (previous_ms == 0U)
  {
    return IMU_MIN_DT_S;
  }

  dt = (float)(now_ms - previous_ms) * 0.001f;
  if (dt < IMU_MIN_DT_S)
  {
    dt = IMU_MIN_DT_S;
  }
  else if (dt > IMU_MAX_DT_S)
  {
    dt = IMU_MAX_DT_S;
  }

  return dt;
}

static void ImuController_ComputeAngles(ImuController *controller)
{
  ImuControllerState *state = &controller->state;
  const float q0 = state->q0;
  const float q1 = state->q1;
  const float q2 = state->q2;
  const float q3 = state->q3;
  float sinp;
  float previous_primary = state->primary_angle_deg;

  state->roll_deg = atan2f(2.0f * ((q0 * q1) + (q2 * q3)),
                           1.0f - (2.0f * ((q1 * q1) + (q2 * q2)))) * IMU_RAD_TO_DEG;

  sinp = 2.0f * ((q0 * q2) - (q3 * q1));
  if (sinp > 1.0f)
  {
    sinp = 1.0f;
  }
  else if (sinp < -1.0f)
  {
    sinp = -1.0f;
  }
  state->pitch_deg = asinf(sinp) * IMU_RAD_TO_DEG;

  state->yaw_deg = atan2f(2.0f * ((q0 * q3) + (q1 * q2)),
                          1.0f - (2.0f * ((q2 * q2) + (q3 * q3)))) * IMU_RAD_TO_DEG;

  state->primary_angle_deg = state->roll_deg - controller->offset_primary_deg;
  state->secondary_angle_deg = state->pitch_deg - controller->offset_secondary_deg;
  state->compensated_angle_deg = state->yaw_deg + controller->offset_compensated_deg;
  state->primary_delta_deg = state->primary_angle_deg - previous_primary;
}

static void ImuController_UpdateMahony(ImuController *controller,
                                       const Mpu6050ScaledSample *sample,
                                       float dt_s)
{
  ImuControllerState *state = &controller->state;
  float ax = -sample->accel_x_mps2;
  float ay = -sample->accel_z_mps2;
  float az = -sample->accel_y_mps2;
  float gx = -sample->gyro_x_dps * IMU_DEG_TO_RAD;
  float gy = -sample->gyro_z_dps * IMU_DEG_TO_RAD;
  float gz = -sample->gyro_y_dps * IMU_DEG_TO_RAD;
  float recip_norm;
  float vx;
  float vy;
  float vz;
  float ex;
  float ey;
  float ez;
  float q0 = state->q0;
  float q1 = state->q1;
  float q2 = state->q2;
  float q3 = state->q3;
  float q_dot0;
  float q_dot1;
  float q_dot2;
  float q_dot3;

  recip_norm = ImuController_InvSqrt((ax * ax) + (ay * ay) + (az * az));
  if (recip_norm == 0.0f)
  {
    return;
  }

  ax *= recip_norm;
  ay *= recip_norm;
  az *= recip_norm;

  vx = 2.0f * ((q1 * q3) - (q0 * q2));
  vy = 2.0f * ((q0 * q1) + (q2 * q3));
  vz = (q0 * q0) - (q1 * q1) - (q2 * q2) + (q3 * q3);

  ex = (ay * vz) - (az * vy);
  ey = (az * vx) - (ax * vz);
  ez = (ax * vy) - (ay * vx);

  state->integral_x += controller->integral_gain * ex * dt_s;
  state->integral_y += controller->integral_gain * ey * dt_s;
  state->integral_z += controller->integral_gain * ez * dt_s;

  gx += (controller->proportional_gain * ex) + state->integral_x;
  gy += (controller->proportional_gain * ey) + state->integral_y;
  gz += (controller->proportional_gain * ez) + state->integral_z;

  q_dot0 = -0.5f * ((q1 * gx) + (q2 * gy) + (q3 * gz));
  q_dot1 =  0.5f * ((q0 * gx) + (q2 * gz) - (q3 * gy));
  q_dot2 =  0.5f * ((q0 * gy) - (q1 * gz) + (q3 * gx));
  q_dot3 =  0.5f * ((q0 * gz) + (q1 * gy) - (q2 * gx));

  q0 += q_dot0 * dt_s;
  q1 += q_dot1 * dt_s;
  q2 += q_dot2 * dt_s;
  q3 += q_dot3 * dt_s;

  recip_norm = ImuController_InvSqrt((q0 * q0) + (q1 * q1) + (q2 * q2) + (q3 * q3));
  if (recip_norm == 0.0f)
  {
    return;
  }

  state->q0 = q0 * recip_norm;
  state->q1 = q1 * recip_norm;
  state->q2 = q2 * recip_norm;
  state->q3 = q3 * recip_norm;
  state->accel_x_mps2 = sample->accel_x_mps2;
  state->accel_y_mps2 = sample->accel_y_mps2;
  state->accel_z_mps2 = sample->accel_z_mps2;
  state->gyro_x_rads = gx;
  state->gyro_y_rads = gy;
  state->gyro_z_rads = gz;
}

static void ImuController_UpdateMotionSafety(ImuController *controller)
{
  ImuControllerState *state = &controller->state;

  if ((fabsf(state->primary_angle_deg) < IMU_SLEEP_ANGLE_LIMIT_DEG) &&
      (fabsf(state->primary_delta_deg) < IMU_SLEEP_DELTA_LIMIT_DEG))
  {
    if (state->stable_sample_count < IMU_SLEEP_STABLE_SAMPLES)
    {
      state->stable_sample_count++;
    }
    else
    {
      state->stable_sample_count = 0;
      if (state->stable_cycle_count < IMU_SLEEP_STABLE_CYCLES)
      {
        state->stable_cycle_count++;
      }
      if (state->stable_cycle_count >= IMU_SLEEP_STABLE_CYCLES)
      {
        state->sleep_request = 1;
        state->stable_cycle_count = 0;
      }
    }
  }
  else
  {
    state->stable_sample_count = 0;
    state->stable_cycle_count = 0;
  }
}

void ImuController_Init(ImuController *controller, MpuController *mpu)
{
  if (controller == NULL)
  {
    return;
  }

  memset(controller, 0, sizeof(*controller));
  controller->mpu = mpu;
  controller->offset_primary_deg = IMU_DEFAULT_PRIMARY_OFFSET;
  controller->offset_secondary_deg = IMU_DEFAULT_SECONDARY_OFFSET;
  controller->offset_compensated_deg = IMU_DEFAULT_COMP_OFFSET;
  controller->proportional_gain = IMU_DEFAULT_KP;
  controller->integral_gain = IMU_DEFAULT_KI;
  ImuController_ResetFilter(controller);
}

void ImuController_ResetFilter(ImuController *controller)
{
  if (controller == NULL)
  {
    return;
  }

  memset(&controller->state, 0, sizeof(controller->state));
  controller->state.q0 = 1.0f;
  controller->state.initialized = 1;
}

HAL_StatusTypeDef ImuController_Update(ImuController *controller)
{
  uint32_t now_ms;
  float dt_s;
  HAL_StatusTypeDef status;

  if ((controller == NULL) || (controller->mpu == NULL))
  {
    return HAL_ERROR;
  }

  status = MpuController_UpdateRaw(controller->mpu);
  if (status != HAL_OK)
  {
    return status;
  }

  now_ms = HAL_GetTick();
  dt_s = ImuController_ClampDt(now_ms, controller->state.last_update_ms);
  controller->state.last_update_ms = now_ms;
  controller->state.last_dt_s = dt_s;

  ImuController_UpdateMahony(controller, &controller->mpu->last_scaled, dt_s);
  ImuController_ComputeAngles(controller);
  controller->state.primary_rate_dps =
      (controller->state.primary_angle_deg - controller->state.last_primary_angle_deg) / dt_s;
  controller->state.last_primary_angle_deg = controller->state.primary_angle_deg;
  controller->state.update_count++;
  ImuController_UpdateMotionSafety(controller);

  return HAL_OK;
}

uint8_t ImuController_ConsumeSleepRequest(ImuController *controller)
{
  uint8_t requested;

  if (controller == NULL)
  {
    return 0;
  }

  requested = controller->state.sleep_request;
  controller->state.sleep_request = 0;
  return requested;
}

const ImuControllerState *ImuController_GetState(const ImuController *controller)
{
  if (controller == NULL)
  {
    return NULL;
  }

  return &controller->state;
}
