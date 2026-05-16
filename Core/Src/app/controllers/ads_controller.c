#include "app/controllers/ads_controller.h"

#include <math.h>

#define ADS_ANGLE_ADC_TO_VOLT       ((float)(3.3f * 0.000244140625f))
#define ADS_ANGLE_VOLTAGE_CENTER    ((float)2.0f)
#define ADS_ANGLE_VOLTAGE_DIVISOR   ((float)30.0f)
#define ADS_ANGLE_RAD_TO_DEG        ((float)57.29577957855229f)
#define ADS_ANGLE_DEFAULT_CENTER    ((float)65.35f)
#define ADS_ANGLE_DEFAULT_ZERO      ((float)0.0f)
#define ADS_ANGLE_DEFAULT_SCALE     ((float)60.0f)

static float AdsController_ClampAsinInput(float value)
{
  if (value > 1.0f)
  {
    return 1.0f;
  }
  if (value < -1.0f)
  {
    return -1.0f;
  }

  return value;
}

static float AdsController_NormalizeRaw(uint16_t raw)
{
  return ((((float)raw * ADS_ANGLE_ADC_TO_VOLT) - ADS_ANGLE_VOLTAGE_CENTER) /
          ADS_ANGLE_VOLTAGE_DIVISOR);
}

void AdsController_Init(AdsController *controller, Ads1118Driver *driver)
{
  if (controller == NULL)
  {
    return;
  }

  controller->driver = driver;
  for (uint32_t i = 0; i < 4; i++)
  {
    controller->last_raw[i] = 0;
  }
  controller->normalized_channel[0] = 0.0f;
  controller->normalized_channel[1] = 0.0f;
  controller->last_raw_angle_deg = 0.0f;
  controller->last_angle_deg = 0.0f;
  controller->last_filtered_angle_deg = 0.0f;
  controller->angle_center_deg = ADS_ANGLE_DEFAULT_CENTER;
  controller->angle_zero_offset_deg = ADS_ANGLE_DEFAULT_ZERO;
  controller->angle_scale_deg = ADS_ANGLE_DEFAULT_SCALE;
  controller->angle_valid = 0U;
}

HAL_StatusTypeDef AdsController_UpdateChannel(AdsController *controller, uint8_t channel)
{
  if ((controller == NULL) || (controller->driver == NULL) || (channel >= 4U))
  {
    return HAL_ERROR;
  }

  return Ads1118Driver_ReadRegisterAveraged(controller->driver, 1, channel, &controller->last_raw[channel]);
}

void AdsController_SetAngleConfig(AdsController *controller,
                                  float center_deg,
                                  float zero_offset_deg,
                                  float scale_deg)
{
  if ((controller == NULL) || !isfinite(center_deg) ||
      !isfinite(zero_offset_deg) || !isfinite(scale_deg))
  {
    return;
  }

  controller->angle_center_deg = center_deg;
  controller->angle_zero_offset_deg = zero_offset_deg;
  controller->angle_scale_deg = scale_deg;
}

HAL_StatusTypeDef AdsController_UpdateAngleEstimate(AdsController *controller)
{
  float raw_angle;
  float centered_angle;
  float scaled_angle;
  HAL_StatusTypeDef status;

  if ((controller == NULL) || (controller->driver == NULL))
  {
    return HAL_ERROR;
  }

  status = AdsController_UpdateChannel(controller, 0U);
  if (status != HAL_OK)
  {
    controller->angle_valid = 0U;
    return status;
  }

  status = AdsController_UpdateChannel(controller, 1U);
  if (status != HAL_OK)
  {
    controller->angle_valid = 0U;
    return status;
  }

  controller->normalized_channel[0] = AdsController_NormalizeRaw(controller->last_raw[0]);
  controller->normalized_channel[1] = AdsController_NormalizeRaw(controller->last_raw[1]);

  raw_angle = atan2f(controller->normalized_channel[1],
                    controller->normalized_channel[0]) * ADS_ANGLE_RAD_TO_DEG;
  raw_angle *= -0.5f;
  if (controller->normalized_channel[1] < 0.0f)
  {
    raw_angle += 180.0f;
  }

  centered_angle = raw_angle - controller->angle_center_deg;
  controller->last_raw_angle_deg = raw_angle;
  controller->last_filtered_angle_deg = raw_angle - controller->angle_zero_offset_deg;

  if ((controller->angle_scale_deg > 0.0001f) && (controller->angle_scale_deg <= 1.0f))
  {
    scaled_angle = (4.25f / controller->angle_scale_deg) * centered_angle;
    centered_angle =
        (285.6f * asinf(AdsController_ClampAsinInput((0.1006f * scaled_angle) - 0.08051f))) +
        (23.93f * asinf(AdsController_ClampAsinInput((1.202f * scaled_angle) + 1.936f))) +
        (2.53f * asinf(AdsController_ClampAsinInput((3.115f * scaled_angle) + 2.917f)));
  }

  if ((controller->angle_scale_deg <= 1.0f) && (centered_angle < 0.0f))
  {
    centered_angle = 0.0f;
  }

  controller->last_angle_deg = centered_angle;
  controller->angle_valid = 1U;
  return HAL_OK;
}
