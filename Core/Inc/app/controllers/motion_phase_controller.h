#ifndef APP_CONTROLLERS_MOTION_PHASE_CONTROLLER_H
#define APP_CONTROLLERS_MOTION_PHASE_CONTROLLER_H

#include "app/controllers/eeprom_controller.h"
#include "app/controllers/imu_controller.h"
#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MOTION_PHASE_FLAG_BYTES ((uint8_t)0x28U)

typedef HAL_StatusTypeDef (*MotionPhaseResponseTx)(uint16_t code,
                                                   const uint8_t *payload,
                                                   uint16_t length);

typedef struct
{
  float off_00;
  float off_04;
  float off_08;
  float off_0c;
  float off_10;
  float off_14;
  float off_18;
  float off_1c;
  float off_20;
  float off_24;
  float off_28;
  float off_2c;
  float off_30;
  float off_34;
  float off_38;
  float off_3c;
  float off_40;
  float off_44;
  float off_48;
  float off_4c;
  float off_50;
  float off_54;
  float off_58;
} MotionPhaseTelemetryConfig;

typedef struct
{
  float a54;
  float a58;
  float a5c;
  float a60;
  float a64;
  float a68;
  uint8_t a6c;
  float a70;
  float a74;
  float a78;
  float a7c;
  uint8_t a80;
  float a84;
  float a88;
  float a8c;
  float a90;
  float a94;
  float a98;
  float a9c;
  uint8_t aa4;
} MotionPhaseRuntimeConfig;

typedef struct
{
  float d_compensated_deg_s;
  float d_primary_deg_s;
  float d_auxiliary_deg_s;
  float d_secondary_deg_s;
  float reserved_10;
  float reserved_14;
  float reserved_18;
  float compensated_angle_deg;
  float primary_angle_deg;
  float auxiliary_angle_deg;
  float secondary_angle_deg;
  float auxiliary_filtered_deg;
  float auxiliary_delta_deg;
  float secondary_duplicate_deg;
  float reserved_38;
  float reserved_3c;
  float d_auxiliary_filtered_deg_s;
} MotionPhaseTelemetry;

typedef struct
{
  const ImuController *imu;
  EepromController *eeprom;
  MotionPhaseResponseTx tx_response;
  MotionPhaseTelemetryConfig telemetry_config;
  MotionPhaseRuntimeConfig runtime_config;
  MotionPhaseTelemetry telemetry;
  uint8_t flags[MOTION_PHASE_FLAG_BYTES];
  float motion_gate_value;
  float motion_gate_threshold;
  float hold_reference_primary_deg;
  float hold_reference_aux_deg;
  float hold_latched_primary_deg;
  float hold_latched_aux_deg;
  float previous_compensated_deg;
  float previous_primary_deg;
  float previous_auxiliary_deg;
  float previous_secondary_deg;
  float previous_auxiliary_filtered_deg;
  float auxiliary_angle_deg;
  float auxiliary_filtered_deg;
  float gyro_x_rads;
  uint32_t update_count;
  uint16_t hold_timer;
  uint16_t limit_timer;
  uint16_t step_event_counter;
  uint8_t hold_armed;
  uint8_t auxiliary_valid;
  uint8_t sensor_motion_flag;
  uint8_t aux_activity_flag;
  uint8_t mode_flag_2e;
  uint8_t mode_flag_2f;
  uint8_t telemetry_pause;
  uint8_t marker_fault;
} MotionPhaseController;

void MotionPhaseController_Init(MotionPhaseController *controller,
                                const ImuController *imu,
                                EepromController *eeprom,
                                MotionPhaseResponseTx tx_response);
void MotionPhaseController_Reset(MotionPhaseController *controller);
void MotionPhaseController_LoadConfigFromEeprom(MotionPhaseController *controller);
void MotionPhaseController_SetAuxiliaryAngle(MotionPhaseController *controller,
                                             float angle_deg,
                                             float filtered_angle_deg);
HAL_StatusTypeDef MotionPhaseController_Update(MotionPhaseController *controller);
const MotionPhaseTelemetry *MotionPhaseController_GetTelemetry(const MotionPhaseController *controller);
uint8_t MotionPhaseController_GetStateCode(const MotionPhaseController *controller);

#ifdef __cplusplus
}
#endif

#endif
