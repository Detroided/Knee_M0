#ifndef APP_DRIVERS_ADS1118_DRIVER_H
#define APP_DRIVERS_ADS1118_DRIVER_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  SPI_HandleTypeDef *hspi;
  GPIO_TypeDef *cs_port;
  uint16_t cs_pin;
} Ads1118Driver;

void Ads1118Driver_Init(Ads1118Driver *driver, SPI_HandleTypeDef *hspi,
                        GPIO_TypeDef *cs_port, uint16_t cs_pin);
HAL_StatusTypeDef Ads1118Driver_ReadRegisterRaw(Ads1118Driver *driver,
                                                uint8_t channel_group,
                                                uint8_t register_index,
                                                uint16_t *value);
HAL_StatusTypeDef Ads1118Driver_ReadRegisterAveraged(Ads1118Driver *driver,
                                                     uint8_t channel_group,
                                                     uint8_t register_index,
                                                     uint16_t *value);

#ifdef __cplusplus
}
#endif

#endif
