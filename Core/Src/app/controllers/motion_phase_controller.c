#include "app/controllers/motion_phase_controller.h"

#include <math.h>
#include <string.h>

#define MOTION_PHASE_DEFAULT_DT_S ((float)0.0065f)
#define MOTION_PHASE_MIN_DT_S     ((float)0.001f)
#define MOTION_PHASE_MAX_DT_S     ((float)0.020f)

#define MOTION_PHASE_RESPONSE_CONTROL ((uint16_t)1U)
#define MOTION_PHASE_RESPONSE_STATE   ((uint16_t)2U)

#define MOTION_PHASE_STATE_BYTE       ((uint8_t)0x00U)
#define MOTION_PHASE_FLAG_LIMIT_ARM   ((uint8_t)0x08U)
#define MOTION_PHASE_FLAG_MOTION_STEP ((uint8_t)0x09U)
#define MOTION_PHASE_FLAG_HOLD        ((uint8_t)0x0AU)
#define MOTION_PHASE_FLAG_LIMIT       ((uint8_t)0x0BU)
#define MOTION_PHASE_FLAG_EVENT_ARM   ((uint8_t)0x0EU)
#define MOTION_PHASE_FLAG_SENSOR      ((uint8_t)0x0FU)
#define MOTION_PHASE_FLAG_MODE_2E     ((uint8_t)0x11U)
#define MOTION_PHASE_FLAG_MODE_2F     ((uint8_t)0x12U)
#define MOTION_PHASE_FLAG_CTRL_A      ((uint8_t)0x16U)
#define MOTION_PHASE_FLAG_CTRL_B      ((uint8_t)0x17U)
#define MOTION_PHASE_FLAG_CTRL_C      ((uint8_t)0x18U)
#define MOTION_PHASE_FLAG_DIRECTION   ((uint8_t)0x24U)

static float MotionPhase_ClampDt(float dt_s)
{
  if ((dt_s < MOTION_PHASE_MIN_DT_S) || (dt_s > MOTION_PHASE_MAX_DT_S))
  {
    return MOTION_PHASE_DEFAULT_DT_S;
  }

  return dt_s;
}

static void MotionPhase_StoreFloatBe(uint8_t *dst, float value)
{
  uint32_t raw;

  memcpy(&raw, &value, sizeof(raw));
  dst[0] = (uint8_t)(raw >> 24);
  dst[1] = (uint8_t)(raw >> 16);
  dst[2] = (uint8_t)(raw >> 8);
  dst[3] = (uint8_t)raw;
}

static float MotionPhase_ReadFloatLe(const uint8_t value[4])
{
  uint32_t raw = ((uint32_t)value[0]) |
                 ((uint32_t)value[1] << 8) |
                 ((uint32_t)value[2] << 16) |
                 ((uint32_t)value[3] << 24);
  float result;

  memcpy(&result, &raw, sizeof(result));
  return result;
}

static float MotionPhase_Powf(float base, float exponent)
{
  float result;

  if (base < 0.0f)
  {
    return 0.0f;
  }

  result = powf(base, exponent);
  if (!isfinite(result))
  {
    return 0.0f;
  }

  return result;
}

static float MotionPhase_Interpolate(float start, float end, float position, float output)
{
  float span = end - start;
  float t;

  if (fabsf(span) < 0.0001f)
  {
    return output;
  }

  t = (position - start) / span;
  if (t < 0.0f)
  {
    t = 0.0f;
  }
  else if (t > 1.0f)
  {
    t = 1.0f;
  }

  return output * t;
}

static void MotionPhase_SendValue(MotionPhaseController *controller,
                                  uint16_t code,
                                  uint8_t type,
                                  float value)
{
  uint8_t payload[5];

  if ((controller == NULL) || (controller->tx_response == NULL))
  {
    return;
  }

  payload[0] = type;
  MotionPhase_StoreFloatBe(&payload[1], value);
  (void)controller->tx_response(code, payload, sizeof(payload));
}

static void MotionPhase_SetDefaultConfig(MotionPhaseController *controller)
{
  MotionPhaseRuntimeConfig *runtime = &controller->runtime_config;
  MotionPhaseTelemetryConfig *cfg = &controller->telemetry_config;

  runtime->a54 = 17.0f;
  runtime->a58 = 0.0f;
  runtime->a5c = 2.0f;
  runtime->a60 = 60.0f;
  runtime->a64 = 0.0f;
  runtime->a68 = 95.0f;
  runtime->a6c = 0U;
  runtime->a70 = 60.0f;
  runtime->a74 = 30.0f;
  runtime->a78 = 0.2f;
  runtime->a7c = 5.0f;
  runtime->a80 = 0U;
  runtime->a84 = 65.0f;
  runtime->a88 = 50.0f;
  runtime->a8c = -1.0f;
  runtime->a90 = 35.0f;
  runtime->a94 = 3.0f;
  runtime->a98 = 35.0f;
  runtime->a9c = 0.0f;
  runtime->aa4 = 0U;

  cfg->off_00 = 20.0f;
  cfg->off_04 = 20.0f;
  cfg->off_08 = 65.0f;
  cfg->off_0c = 5.0f;
  cfg->off_10 = 180.0f;
  cfg->off_14 = 90.0f;
  cfg->off_18 = -18.5f;
  cfg->off_1c = -6.0f;
  cfg->off_20 = 65.0f;
  cfg->off_24 = 0.0f;
  cfg->off_28 = 0.0f;
  cfg->off_2c = 90.0f;
  cfg->off_30 = 200.0f;
  cfg->off_34 = 0.0f;
  cfg->off_38 = 50.0f;
  cfg->off_3c = 0.0f;
  cfg->off_40 = 0.0f;
  cfg->off_44 = 50.0f;
  cfg->off_48 = 35.0f;
  cfg->off_4c = 85.0f;
  cfg->off_50 = 5.0f;
  cfg->off_54 = 50.0f;
  cfg->off_58 = -6.0f;
}

static void MotionPhase_ReadFloatParam(MotionPhaseController *controller,
                                       uint16_t param_id,
                                       float *target)
{
  uint8_t bytes[4];
  float value;

  if ((controller == NULL) || (controller->eeprom == NULL) || (target == NULL))
  {
    return;
  }

  if (EepromController_ReadParam32(controller->eeprom, param_id, bytes) != HAL_OK)
  {
    return;
  }

  value = MotionPhase_ReadFloatLe(bytes);
  if (isfinite(value))
  {
    *target = value;
  }
}

static void MotionPhase_ReadU8Param(MotionPhaseController *controller,
                                    uint16_t param_id,
                                    uint8_t *target,
                                    uint8_t max_value)
{
  uint8_t value;

  if ((controller == NULL) || (controller->eeprom == NULL) || (target == NULL))
  {
    return;
  }

  if (EepromController_ReadParam8(controller->eeprom, param_id, &value) != HAL_OK)
  {
    return;
  }

  if (value <= max_value)
  {
    *target = value;
  }
}

static uint8_t MotionPhase_IsBlocked(const MotionPhaseController *controller)
{
  return (uint8_t)((controller->flags[MOTION_PHASE_FLAG_MODE_2E] == 1U) ||
                   (controller->flags[MOTION_PHASE_FLAG_SENSOR] == 1U) ||
                   (controller->flags[MOTION_PHASE_FLAG_MODE_2F] == 1U));
}

static void MotionPhase_UpdateTelemetry(MotionPhaseController *controller, float dt_s)
{
  const ImuControllerState *imu = ImuController_GetState(controller->imu);
  MotionPhaseTelemetry *telemetry = &controller->telemetry;
  float auxiliary = controller->auxiliary_valid != 0U ? controller->auxiliary_angle_deg : 0.0f;
  float auxiliary_filtered =
      controller->auxiliary_valid != 0U ? controller->auxiliary_filtered_deg : auxiliary;

  telemetry->compensated_angle_deg = imu->compensated_angle_deg;
  telemetry->primary_angle_deg = imu->primary_angle_deg;
  telemetry->auxiliary_angle_deg = auxiliary;
  telemetry->secondary_angle_deg = imu->secondary_angle_deg;
  telemetry->auxiliary_filtered_deg = auxiliary_filtered;
  telemetry->auxiliary_delta_deg = auxiliary - controller->previous_auxiliary_deg;
  telemetry->secondary_duplicate_deg = imu->secondary_angle_deg;
  telemetry->reserved_38 = 0.0f;
  telemetry->reserved_3c = 0.0f;

  telemetry->d_compensated_deg_s =
      (telemetry->compensated_angle_deg - controller->previous_compensated_deg) / dt_s;
  telemetry->d_primary_deg_s =
      (telemetry->primary_angle_deg - controller->previous_primary_deg) / dt_s;
  telemetry->d_auxiliary_deg_s =
      (telemetry->auxiliary_angle_deg - controller->previous_auxiliary_deg) / dt_s;
  telemetry->d_secondary_deg_s =
      (telemetry->secondary_angle_deg - controller->previous_secondary_deg) / dt_s;
  telemetry->d_auxiliary_filtered_deg_s =
      (telemetry->auxiliary_filtered_deg - controller->previous_auxiliary_filtered_deg) / dt_s;

  controller->previous_compensated_deg = telemetry->compensated_angle_deg;
  controller->previous_primary_deg = telemetry->primary_angle_deg;
  controller->previous_auxiliary_deg = telemetry->auxiliary_angle_deg;
  controller->previous_secondary_deg = telemetry->secondary_angle_deg;
  controller->previous_auxiliary_filtered_deg = telemetry->auxiliary_filtered_deg;
  controller->gyro_x_rads = imu->gyro_x_rads;
}

static void MotionPhase_UpdateStateLimitFlags(MotionPhaseController *controller)
{
  MotionPhaseTelemetry *state = &controller->telemetry;
  MotionPhaseTelemetryConfig *cfg = &controller->telemetry_config;
  MotionPhaseRuntimeConfig *runtime = &controller->runtime_config;
  uint16_t limit_target = (uint16_t)runtime->aa4 * 500U;

  if (MotionPhase_IsBlocked(controller) != 0U)
  {
    return;
  }

  if ((cfg->off_14 < 89.0f) &&
      (state->primary_angle_deg > cfg->off_14) &&
      (state->compensated_angle_deg > 2.0f) &&
      (state->auxiliary_angle_deg > 2.0f))
  {
    controller->limit_timer = 0U;
    controller->flags[MOTION_PHASE_FLAG_HOLD] = 0U;
    controller->flags[MOTION_PHASE_FLAG_LIMIT] = 1U;
    controller->flags[MOTION_PHASE_FLAG_LIMIT_ARM] = 0U;
    controller->flags[MOTION_PHASE_STATE_BYTE] = 2U;
  }

  if ((state->compensated_angle_deg > cfg->off_4c) &&
      (state->auxiliary_angle_deg > 2.0f))
  {
    if ((runtime->aa4 == 0U) || (controller->limit_timer >= limit_target))
    {
      controller->limit_timer = 0U;
      controller->flags[MOTION_PHASE_FLAG_HOLD] = 0U;
      controller->flags[MOTION_PHASE_FLAG_LIMIT] = 1U;
      controller->flags[MOTION_PHASE_FLAG_LIMIT_ARM] = 0U;
      controller->flags[MOTION_PHASE_STATE_BYTE] = 2U;
    }
    else
    {
      controller->limit_timer++;
    }
  }
  else
  {
    controller->limit_timer = 0U;
  }

  if ((state->compensated_angle_deg <= cfg->off_20) ||
      (state->auxiliary_filtered_deg >= runtime->a8c))
  {
    controller->flags[MOTION_PHASE_FLAG_LIMIT] = 0U;
  }
}

static void MotionPhase_UpdateHoldStateMachine(MotionPhaseController *controller)
{
  MotionPhaseTelemetry *state = &controller->telemetry;
  MotionPhaseTelemetryConfig *cfg = &controller->telemetry_config;
  MotionPhaseRuntimeConfig *runtime = &controller->runtime_config;
  uint16_t hold_target = runtime->a6c == 0U ? (uint16_t)(0xFFU * 250U)
                                            : (uint16_t)runtime->a6c * 250U;

  if (MotionPhase_IsBlocked(controller) != 0U)
  {
    return;
  }

  if ((state->compensated_angle_deg < cfg->off_4c) &&
      (state->auxiliary_angle_deg < runtime->a68) &&
      (state->primary_angle_deg < 12.0f))
  {
    if ((controller->hold_armed == 0U) &&
        (state->auxiliary_angle_deg > cfg->off_0c))
    {
      controller->hold_armed = 1U;
      controller->hold_timer = 0U;
      controller->hold_reference_primary_deg = state->primary_angle_deg;
      controller->hold_reference_aux_deg = state->auxiliary_angle_deg;
    }

    if (controller->hold_armed != 0U)
    {
      if ((fabsf(state->auxiliary_angle_deg - controller->hold_reference_aux_deg) >= 1.25f) ||
          (fabsf(state->primary_angle_deg - controller->hold_reference_primary_deg) >= 1.25f))
      {
        controller->hold_armed = 0U;
        controller->hold_timer = 0U;
      }
      else if (controller->hold_timer < hold_target)
      {
        controller->hold_timer++;
      }

      if ((controller->hold_timer >= hold_target) &&
          (controller->flags[MOTION_PHASE_FLAG_MOTION_STEP] == 0U) &&
          (controller->flags[MOTION_PHASE_FLAG_LIMIT] == 0U))
      {
        controller->flags[MOTION_PHASE_STATE_BYTE] = 0U;
        controller->flags[MOTION_PHASE_FLAG_HOLD] = 1U;
        controller->hold_latched_primary_deg = state->primary_angle_deg;
        controller->hold_latched_aux_deg = state->auxiliary_angle_deg;
        controller->hold_timer = 0U;
      }
    }
  }
  else
  {
    controller->hold_timer = 0U;
    controller->hold_armed = 0U;
  }

  if ((controller->flags[MOTION_PHASE_FLAG_HOLD] == 1U) &&
      (fabsf(state->primary_angle_deg - controller->hold_latched_primary_deg) >= 10.0f))
  {
    controller->flags[MOTION_PHASE_FLAG_HOLD] = 0U;
    controller->hold_armed = 0U;
  }
}

static void MotionPhase_UpdateMotionStateMachine(MotionPhaseController *controller)
{
  MotionPhaseTelemetry *state = &controller->telemetry;
  MotionPhaseTelemetryConfig *cfg = &controller->telemetry_config;
  MotionPhaseRuntimeConfig *runtime = &controller->runtime_config;
  float phase_threshold;

  if (MotionPhase_IsBlocked(controller) != 0U)
  {
    return;
  }

  phase_threshold = cfg->off_1c > 0.0f ? cfg->off_1c * 0.5f : cfg->off_1c * 2.0f;
  if (state->primary_angle_deg > phase_threshold)
  {
    if ((state->auxiliary_angle_deg > runtime->a54) ||
        (state->primary_angle_deg < 2.0f))
    {
      controller->flags[MOTION_PHASE_FLAG_MOTION_STEP] = 0U;
    }
  }
  else if (state->primary_angle_deg > 2.0f)
  {
    controller->flags[MOTION_PHASE_FLAG_MOTION_STEP] = 0U;
  }

  if ((controller->flags[MOTION_PHASE_FLAG_MOTION_STEP] == 2U) &&
      ((state->auxiliary_angle_deg > runtime->a54) ||
       (state->primary_angle_deg > 0.0f)))
  {
    controller->flags[MOTION_PHASE_FLAG_MOTION_STEP] = 0U;
  }

  if (cfg->off_34 >= 1.0f)
  {
    controller->flags[MOTION_PHASE_FLAG_DIRECTION] = 0U;
    controller->motion_gate_value = 17.5f;
  }
  else if (fabsf(cfg->off_34) > 1.0f)
  {
    controller->flags[MOTION_PHASE_FLAG_DIRECTION] = 0U;
    controller->motion_gate_value = 0.0f;
  }
  else
  {
    controller->flags[MOTION_PHASE_FLAG_DIRECTION] = 1U;
    controller->motion_gate_value = 0.0f;
  }
  controller->motion_gate_threshold = cfg->off_18;

  if (controller->flags[MOTION_PHASE_FLAG_MOTION_STEP] == 1U)
  {
    if (controller->flags[MOTION_PHASE_FLAG_DIRECTION] == 0U)
    {
      if (((state->d_auxiliary_deg_s >= 0.0f) &&
           (state->d_primary_deg_s <= controller->motion_gate_value) &&
           (state->d_auxiliary_filtered_deg_s >= 0.0f)) ||
          ((state->primary_angle_deg < 0.5f) &&
           (controller->gyro_x_rads <= runtime->a78)))
      {
        if ((controller->flags[MOTION_PHASE_FLAG_HOLD] == 0U) &&
            (controller->flags[MOTION_PHASE_FLAG_LIMIT] == 0U))
        {
          controller->flags[MOTION_PHASE_STATE_BYTE] = 7U;
          controller->flags[MOTION_PHASE_FLAG_MOTION_STEP] = 2U;
        }
      }
    }
    else
    {
      if ((state->d_primary_deg_s > 0.0f) ||
          (state->d_auxiliary_filtered_deg_s < 0.0f) ||
          (state->d_auxiliary_deg_s <= 0.0f))
      {
        controller->flags[MOTION_PHASE_FLAG_MOTION_STEP] = 0U;
      }
    }
  }

  if ((state->compensated_angle_deg < cfg->off_58) &&
      (state->primary_angle_deg < cfg->off_1c) &&
      (state->auxiliary_filtered_deg < runtime->a8c) &&
      (state->d_primary_deg_s < controller->motion_gate_threshold) &&
      (controller->flags[MOTION_PHASE_FLAG_MOTION_STEP] == 0U))
  {
    controller->flags[MOTION_PHASE_FLAG_LIMIT_ARM] = 1U;
    controller->flags[MOTION_PHASE_FLAG_MOTION_STEP] = 1U;
    controller->flags[MOTION_PHASE_FLAG_HOLD] = 0U;
    controller->step_event_counter = 0U;
    controller->flags[MOTION_PHASE_STATE_BYTE] = 1U;
  }

  if (state->primary_angle_deg < cfg->off_1c)
  {
    controller->flags[MOTION_PHASE_FLAG_EVENT_ARM] = 1U;
  }

  if ((state->auxiliary_angle_deg > runtime->a54) &&
      (controller->flags[MOTION_PHASE_FLAG_MOTION_STEP] == 0U))
  {
    controller->flags[MOTION_PHASE_FLAG_LIMIT_ARM] = 0U;
  }

  if ((state->primary_angle_deg > 2.0f) &&
      (controller->flags[MOTION_PHASE_FLAG_EVENT_ARM] == 1U))
  {
    controller->step_event_counter++;
    controller->flags[MOTION_PHASE_FLAG_EVENT_ARM] = 0U;
  }
}

static void MotionPhase_SendComputedStateValue(MotionPhaseController *controller)
{
  MotionPhaseTelemetry *state = &controller->telemetry;
  MotionPhaseTelemetryConfig *cfg = &controller->telemetry_config;
  MotionPhaseRuntimeConfig *runtime = &controller->runtime_config;
  uint8_t type = 2U;
  float value;

  if (state->auxiliary_angle_deg >= runtime->a68)
  {
    type = 0U;
    value = cfg->off_24;
  }
  else
  {
    value = cfg->off_38 + MotionPhase_Powf(state->auxiliary_angle_deg, runtime->a58);
  }

  MotionPhase_SendValue(controller, MOTION_PHASE_RESPONSE_STATE, type, value);
}

static void MotionPhase_SendStateMachineValueResponse(MotionPhaseController *controller)
{
  MotionPhaseTelemetry *state = &controller->telemetry;
  MotionPhaseTelemetryConfig *cfg = &controller->telemetry_config;
  MotionPhaseRuntimeConfig *runtime = &controller->runtime_config;
  uint8_t state_code = controller->flags[MOTION_PHASE_STATE_BYTE];
  uint8_t type = state_code;
  float value = cfg->off_24;

  switch (state_code)
  {
    case 0U:
      if (state->auxiliary_angle_deg <= runtime->a68)
      {
        value = 90.0f;
      }
      break;

    case 1U:
      type = 0U;
      value = cfg->off_24;
      break;

    case 2U:
    case 9U:
      value = cfg->off_24;
      break;

    case 3U:
      MotionPhase_SendComputedStateValue(controller);
      return;

    case 5U:
      if (state->auxiliary_angle_deg <= runtime->a68)
      {
        type = 2U;
        value = cfg->off_38 + MotionPhase_Powf(state->auxiliary_angle_deg, runtime->a58);
      }
      else
      {
        type = 0U;
        value = cfg->off_24;
      }
      break;

    case 7U:
      value = state->auxiliary_angle_deg >= runtime->a68 ? cfg->off_24 : runtime->a60;
      break;

    case 10U:
      value = 90.0f;
      break;

    default:
      type = state_code;
      value = cfg->off_24;
      break;
  }

  MotionPhase_SendValue(controller, MOTION_PHASE_RESPONSE_STATE, type, value);
}

static void MotionPhase_EvaluateAndSendControlResponse(MotionPhaseController *controller)
{
  MotionPhaseTelemetry *state = &controller->telemetry;
  MotionPhaseTelemetryConfig *cfg = &controller->telemetry_config;
  MotionPhaseRuntimeConfig *runtime = &controller->runtime_config;
  float value;
  uint8_t type;

  if (MotionPhase_IsBlocked(controller) != 0U)
  {
    MotionPhase_SendValue(controller, MOTION_PHASE_RESPONSE_CONTROL, 1U, cfg->off_44);
    return;
  }

  if (controller->telemetry_pause != 0U)
  {
    MotionPhase_SendValue(controller, MOTION_PHASE_RESPONSE_CONTROL, 0U, cfg->off_28);
    return;
  }

  if (state->auxiliary_angle_deg > runtime->a68)
  {
    MotionPhase_SendValue(controller, MOTION_PHASE_RESPONSE_CONTROL, 3U, cfg->off_28);
    return;
  }

  if ((controller->flags[MOTION_PHASE_FLAG_CTRL_A] == 0U) &&
      (controller->flags[MOTION_PHASE_FLAG_CTRL_B] == 1U) &&
      ((state->d_primary_deg_s <= 0.0f) || (state->d_auxiliary_deg_s <= 0.0f)) &&
      (state->auxiliary_angle_deg < cfg->off_50))
  {
    controller->flags[MOTION_PHASE_FLAG_CTRL_A] = 1U;
  }

  if ((controller->flags[MOTION_PHASE_FLAG_CTRL_B] == 1U) &&
      (controller->flags[MOTION_PHASE_FLAG_CTRL_A] == 0U))
  {
    value = state->auxiliary_angle_deg >= cfg->off_48
                ? cfg->off_44
                : MotionPhase_Interpolate(runtime->a64, cfg->off_48,
                                          state->auxiliary_angle_deg, cfg->off_44);
    type = state->auxiliary_angle_deg >= cfg->off_48 ? 1U : 5U;
    MotionPhase_SendValue(controller, MOTION_PHASE_RESPONSE_CONTROL, type, value);
    return;
  }

  if ((controller->flags[MOTION_PHASE_FLAG_CTRL_A] == 1U) &&
      (controller->flags[MOTION_PHASE_FLAG_CTRL_B] == 0U))
  {
    value = state->auxiliary_angle_deg >= runtime->a90
                ? cfg->off_44
                : MotionPhase_Interpolate(runtime->a94, runtime->a90,
                                          state->auxiliary_angle_deg, cfg->off_44);
    type = state->auxiliary_angle_deg >= runtime->a90 ? 1U : 5U;
    MotionPhase_SendValue(controller, MOTION_PHASE_RESPONSE_CONTROL, type, value);
    return;
  }

  if (state->auxiliary_angle_deg > runtime->a54)
  {
    controller->flags[MOTION_PHASE_FLAG_CTRL_C] = 1U;
  }
  if ((controller->flags[MOTION_PHASE_FLAG_CTRL_C] == 1U) &&
      (state->auxiliary_angle_deg < 2.0f))
  {
    controller->flags[MOTION_PHASE_FLAG_CTRL_C] = 2U;
  }
  if ((controller->flags[MOTION_PHASE_FLAG_CTRL_C] == 2U) &&
      (state->auxiliary_angle_deg > 2.0f))
  {
    controller->flags[MOTION_PHASE_FLAG_CTRL_C] = 0U;
  }

  if (state->auxiliary_angle_deg >= runtime->a98)
  {
    MotionPhase_SendValue(controller, MOTION_PHASE_RESPONSE_CONTROL, 1U, cfg->off_44);
    return;
  }

  value = controller->flags[MOTION_PHASE_FLAG_CTRL_C] == 0U
              ? MotionPhase_Interpolate(runtime->a94, runtime->a90,
                                        state->auxiliary_angle_deg, cfg->off_44)
              : MotionPhase_Interpolate(runtime->a9c, runtime->a98,
                                        state->auxiliary_angle_deg, cfg->off_44);
  MotionPhase_SendValue(controller, MOTION_PHASE_RESPONSE_CONTROL, 5U, value);
}

static void MotionPhase_SendStateTelemetry(MotionPhaseController *controller)
{
  MotionPhaseTelemetry *state = &controller->telemetry;
  uint8_t in_motion_window;

  if (controller->marker_fault != 0U)
  {
    MotionPhase_SendValue(controller, MOTION_PHASE_RESPONSE_STATE, 4U, 90.0f);
    MotionPhase_SendValue(controller, MOTION_PHASE_RESPONSE_CONTROL, 0U, 0.0f);
    return;
  }

  in_motion_window = (uint8_t)((state->primary_angle_deg > -90.0f) &&
                               (state->primary_angle_deg < 90.0f) &&
                               (fabsf(state->secondary_angle_deg) < 30.0f));

  if ((in_motion_window != 0U) ||
      (controller->flags[MOTION_PHASE_STATE_BYTE] == 9U) ||
      ((controller->flags[MOTION_PHASE_STATE_BYTE] & 0xFDU) == 8U))
  {
    if ((controller->sensor_motion_flag == 1U) || (controller->aux_activity_flag == 1U))
    {
      controller->flags[MOTION_PHASE_FLAG_SENSOR] = 1U;
      controller->flags[MOTION_PHASE_STATE_BYTE] = 8U;
    }
    else
    {
      controller->flags[MOTION_PHASE_FLAG_SENSOR] = 0U;
    }

    controller->flags[MOTION_PHASE_FLAG_MODE_2E] = controller->mode_flag_2e == 1U ? 1U : 0U;
    if (controller->flags[MOTION_PHASE_FLAG_MODE_2E] != 0U)
    {
      controller->flags[MOTION_PHASE_STATE_BYTE] = 9U;
    }

    controller->flags[MOTION_PHASE_FLAG_MODE_2F] = controller->mode_flag_2f == 1U ? 1U : 0U;
    if (controller->flags[MOTION_PHASE_FLAG_MODE_2F] != 0U)
    {
      controller->flags[MOTION_PHASE_STATE_BYTE] = 10U;
    }

    if ((controller->flags[MOTION_PHASE_FLAG_MODE_2E] == 0U) &&
        (controller->flags[MOTION_PHASE_FLAG_SENSOR] != 1U) &&
        (controller->flags[MOTION_PHASE_FLAG_MODE_2F] == 0U) &&
        (state->compensated_angle_deg < controller->telemetry_config.off_4c) &&
        (controller->flags[MOTION_PHASE_FLAG_LIMIT] == 0U) &&
        (controller->flags[MOTION_PHASE_FLAG_HOLD] == 0U) &&
        (controller->flags[MOTION_PHASE_FLAG_MOTION_STEP] == 0U))
    {
      controller->flags[MOTION_PHASE_STATE_BYTE] = 3U;
    }

    MotionPhase_UpdateStateLimitFlags(controller);
    MotionPhase_UpdateHoldStateMachine(controller);
    MotionPhase_UpdateMotionStateMachine(controller);
    MotionPhase_EvaluateAndSendControlResponse(controller);
    MotionPhase_SendStateMachineValueResponse(controller);
    return;
  }

  MotionPhase_SendComputedStateValue(controller);
}

void MotionPhaseController_Init(MotionPhaseController *controller,
                                const ImuController *imu,
                                EepromController *eeprom,
                                MotionPhaseResponseTx tx_response)
{
  if (controller == NULL)
  {
    return;
  }

  memset(controller, 0, sizeof(*controller));
  controller->imu = imu;
  controller->eeprom = eeprom;
  controller->tx_response = tx_response;
  MotionPhase_SetDefaultConfig(controller);
  MotionPhaseController_LoadConfigFromEeprom(controller);
}

void MotionPhaseController_Reset(MotionPhaseController *controller)
{
  if (controller == NULL)
  {
    return;
  }

  memset(&controller->telemetry, 0, sizeof(controller->telemetry));
  memset(controller->flags, 0, sizeof(controller->flags));
  controller->motion_gate_value = 0.0f;
  controller->motion_gate_threshold = 0.0f;
  controller->hold_reference_primary_deg = 0.0f;
  controller->hold_reference_aux_deg = 0.0f;
  controller->hold_latched_primary_deg = 0.0f;
  controller->hold_latched_aux_deg = 0.0f;
  controller->previous_compensated_deg = 0.0f;
  controller->previous_primary_deg = 0.0f;
  controller->previous_auxiliary_deg = 0.0f;
  controller->previous_secondary_deg = 0.0f;
  controller->previous_auxiliary_filtered_deg = 0.0f;
  controller->update_count = 0U;
  controller->hold_timer = 0U;
  controller->limit_timer = 0U;
  controller->step_event_counter = 0U;
  controller->hold_armed = 0U;
}

void MotionPhaseController_LoadConfigFromEeprom(MotionPhaseController *controller)
{
  MotionPhaseRuntimeConfig *runtime;

  if (controller == NULL)
  {
    return;
  }

  runtime = &controller->runtime_config;

  if (controller->eeprom != NULL)
  {
    if (EepromController_Probe(controller->eeprom) == HAL_OK)
    {
      if (EepromController_ReadMarkers(controller->eeprom) == HAL_OK)
      {
        controller->marker_fault =
            (uint8_t)((controller->eeprom->marker_w == (uint8_t)'w') ||
                      (controller->eeprom->marker_f == (uint8_t)'f') ||
                      (controller->eeprom->marker_u != (uint8_t)'U'));
      }

      MotionPhase_ReadFloatParam(controller, 0x19U, &runtime->a54);
      MotionPhase_ReadFloatParam(controller, 0x1AU, &runtime->a58);
      MotionPhase_ReadFloatParam(controller, 0x1BU, &runtime->a5c);
      MotionPhase_ReadFloatParam(controller, 0x1CU, &runtime->a60);
      MotionPhase_ReadFloatParam(controller, 0x1FU, &runtime->a64);
      MotionPhase_ReadFloatParam(controller, 0x20U, &runtime->a70);
      MotionPhase_ReadU8Param(controller, 0x21U, &runtime->a6c, 10U);
      MotionPhase_ReadFloatParam(controller, 0x22U, &runtime->a68);
      MotionPhase_ReadFloatParam(controller, 0x23U, &runtime->a74);
      MotionPhase_ReadU8Param(controller, 0x24U, &runtime->a80, 99U);
      MotionPhase_ReadFloatParam(controller, 0x27U, &runtime->a78);
      MotionPhase_ReadFloatParam(controller, 0x28U, &runtime->a7c);
      MotionPhase_ReadFloatParam(controller, 0x29U, &runtime->a84);
      MotionPhase_ReadFloatParam(controller, 0x2AU, &runtime->a88);
      MotionPhase_ReadFloatParam(controller, 0x2BU, &runtime->a8c);
      MotionPhase_ReadFloatParam(controller, 0x2CU, &runtime->a90);
      MotionPhase_ReadFloatParam(controller, 0x2DU, &runtime->a94);
      MotionPhase_ReadFloatParam(controller, 0x2FU, &runtime->a98);
      MotionPhase_ReadFloatParam(controller, 0x30U, &runtime->a9c);
      MotionPhase_ReadU8Param(controller, 0x15U, &runtime->aa4, 10U);
    }
  }
}

void MotionPhaseController_SetAuxiliaryAngle(MotionPhaseController *controller,
                                             float angle_deg,
                                             float filtered_angle_deg)
{
  if ((controller == NULL) || !isfinite(angle_deg) || !isfinite(filtered_angle_deg))
  {
    return;
  }

  controller->auxiliary_angle_deg = angle_deg;
  controller->auxiliary_filtered_deg = filtered_angle_deg;
  controller->auxiliary_valid = 1U;
}

HAL_StatusTypeDef MotionPhaseController_Update(MotionPhaseController *controller)
{
  const ImuControllerState *imu;
  float dt_s;

  if ((controller == NULL) || (controller->imu == NULL))
  {
    return HAL_ERROR;
  }

  imu = ImuController_GetState(controller->imu);
  if ((imu == NULL) || (imu->initialized == 0U))
  {
    return HAL_ERROR;
  }

  dt_s = MotionPhase_ClampDt(imu->last_dt_s);
  MotionPhase_UpdateTelemetry(controller, dt_s);
  MotionPhase_SendStateTelemetry(controller);
  controller->update_count++;

  return HAL_OK;
}

const MotionPhaseTelemetry *MotionPhaseController_GetTelemetry(const MotionPhaseController *controller)
{
  if (controller == NULL)
  {
    return NULL;
  }

  return &controller->telemetry;
}

uint8_t MotionPhaseController_GetStateCode(const MotionPhaseController *controller)
{
  if (controller == NULL)
  {
    return 0U;
  }

  return controller->flags[MOTION_PHASE_STATE_BYTE];
}
