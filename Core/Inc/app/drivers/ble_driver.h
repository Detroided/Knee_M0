#ifndef APP_DRIVERS_BLE_DRIVER_H
#define APP_DRIVERS_BLE_DRIVER_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  UART_HandleTypeDef *huart;
  GPIO_TypeDef *power_port;
  uint16_t power_pin;
  GPIO_TypeDef *reset_port;
  uint16_t reset_pin;
} BleDriver;

void BleDriver_Init(BleDriver *driver, UART_HandleTypeDef *huart,
                    GPIO_TypeDef *power_port, uint16_t power_pin,
                    GPIO_TypeDef *reset_port, uint16_t reset_pin);
void BleDriver_SetPower(BleDriver *driver, GPIO_PinState state);
void BleDriver_SetReset(BleDriver *driver, GPIO_PinState state);
HAL_StatusTypeDef BleDriver_Transmit(BleDriver *driver, const uint8_t *data,
                                     uint16_t length, uint32_t timeout_ms);
HAL_StatusTypeDef BleDriver_ReceiveDma(BleDriver *driver, uint8_t *data, uint16_t length);

#ifdef __cplusplus
}
#endif

#endif
