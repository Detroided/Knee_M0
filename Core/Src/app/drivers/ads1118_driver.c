#include "app/drivers/ads1118_driver.h"

void Ads1118Driver_Init(Ads1118Driver *driver, SPI_HandleTypeDef *hspi,
                        GPIO_TypeDef *cs_port, uint16_t cs_pin)
{
  if (driver == NULL)
  {
    return;
  }

  driver->hspi = hspi;
  driver->cs_port = cs_port;
  driver->cs_pin = cs_pin;

  if (cs_port != NULL)
  {
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
  }
}

HAL_StatusTypeDef Ads1118Driver_ReadRegisterRaw(Ads1118Driver *driver,
                                                uint8_t channel_group,
                                                uint8_t register_index,
                                                uint16_t *value)
{
  uint8_t tx[3] = {0};
  uint8_t rx[3] = {0};

  if ((driver == NULL) || (driver->hspi == NULL) || (driver->cs_port == NULL) || (value == NULL))
  {
    return HAL_ERROR;
  }

  tx[0] = (uint8_t)(((channel_group << 7) | (register_index << 6)) + 0x20);
  tx[1] = 0x01;
  tx[2] = 0x00;

  HAL_GPIO_WritePin(driver->cs_port, driver->cs_pin, GPIO_PIN_RESET);
  for (volatile uint32_t i = 0; i < 0x48; i++)
  {
  }

  HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(driver->hspi, tx, rx, sizeof(tx), 100);

  for (volatile uint32_t i = 0; i < 0x48; i++)
  {
  }
  HAL_GPIO_WritePin(driver->cs_port, driver->cs_pin, GPIO_PIN_SET);

  if (status != HAL_OK)
  {
    return status;
  }

  *value = (uint16_t)(((uint16_t)(rx[1] & 0x0F) << 8) | rx[2]);
  return HAL_OK;
}

HAL_StatusTypeDef Ads1118Driver_ReadRegisterAveraged(Ads1118Driver *driver,
                                                     uint8_t channel_group,
                                                     uint8_t register_index,
                                                     uint16_t *value)
{
  uint32_t sum = 0;

  if (value == NULL)
  {
    return HAL_ERROR;
  }

  for (uint32_t i = 0; i < 3; i++)
  {
    uint16_t raw = 0;
    HAL_StatusTypeDef status = Ads1118Driver_ReadRegisterRaw(driver, channel_group, register_index, &raw);
    if (status != HAL_OK)
    {
      return status;
    }
    sum += raw;
  }

  *value = (uint16_t)(sum / 3U);
  return HAL_OK;
}
