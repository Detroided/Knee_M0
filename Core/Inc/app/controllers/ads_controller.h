#ifndef APP_CONTROLLERS_ADS_CONTROLLER_H
#define APP_CONTROLLERS_ADS_CONTROLLER_H

#include "app/drivers/ads1118_driver.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  Ads1118Driver *driver;
  uint16_t last_raw[4];
  float normalized_channel[2];
  float last_raw_angle_deg;
  float last_angle_deg;
  float last_filtered_angle_deg;
  float angle_center_deg;
  float angle_zero_offset_deg;
  float angle_scale_deg;
  uint8_t angle_valid;
} AdsController;

void AdsController_Init(AdsController *controller, Ads1118Driver *driver);
HAL_StatusTypeDef AdsController_UpdateChannel(AdsController *controller, uint8_t channel);
void AdsController_SetAngleConfig(AdsController *controller,
                                  float center_deg,
                                  float zero_offset_deg,
                                  float scale_deg);
HAL_StatusTypeDef AdsController_UpdateAngleEstimate(AdsController *controller);

#ifdef __cplusplus
}
#endif

#endif
