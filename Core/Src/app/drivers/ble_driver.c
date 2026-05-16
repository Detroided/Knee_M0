#include "app/drivers/ble_driver.h"

void BleDriver_Init(BleDriver *driver, UART_HandleTypeDef *huart,
                    GPIO_TypeDef *power_port, uint16_t power_pin,
                    GPIO_TypeDef *reset_port, uint16_t reset_pin)
{
  if (driver == NULL)
  {
    return;
  }

  driver->huart = huart;
  driver->power_port = power_port;
  driver->power_pin = power_pin;
  driver->reset_port = reset_port;
  driver->reset_pin = reset_pin;
}

void BleDriver_SetPower(BleDriver *driver, GPIO_PinState state)
{
  if ((driver == NULL) || (driver->power_port == NULL))
  {
    return;
  }

  HAL_GPIO_WritePin(driver->power_port, driver->power_pin, state);
}

void BleDriver_SetReset(BleDriver *driver, GPIO_PinState state)
{
  if ((driver == NULL) || (driver->reset_port == NULL))
  {
    return;
  }

  HAL_GPIO_WritePin(driver->reset_port, driver->reset_pin, state);
}

HAL_StatusTypeDef BleDriver_Transmit(BleDriver *driver, const uint8_t *data,
                                     uint16_t length, uint32_t timeout_ms)
{
  if ((driver == NULL) || (driver->huart == NULL) || (data == NULL))
  {
    return HAL_ERROR;
  }

  return HAL_UART_Transmit(driver->huart, (uint8_t *)data, length, timeout_ms);
}

HAL_StatusTypeDef BleDriver_ReceiveDma(BleDriver *driver, uint8_t *data, uint16_t length)
{
  if ((driver == NULL) || (driver->huart == NULL) || (data == NULL))
  {
    return HAL_ERROR;
  }

  return HAL_UART_Receive_DMA(driver->huart, data, length);
}
