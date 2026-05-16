#ifndef APP_CONTROLLERS_BLE_CONTROLLER_H
#define APP_CONTROLLERS_BLE_CONTROLLER_H

#include "app/app_protocol.h"
#include "app/controllers/eeprom_controller.h"
#include "app/drivers/ble_driver.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  BleDriver *driver;
  EepromController *eeprom;
  uint16_t last_command;
  uint16_t last_param_id;
  HAL_StatusTypeDef last_status;
} BleController;

void BleController_Init(BleController *controller, BleDriver *driver);
void BleController_AttachEeprom(BleController *controller, EepromController *eeprom);
void BleController_SetEnabled(BleController *controller, uint8_t enabled);
void BleController_HandlePacket(BleController *controller, AppProtocolChannel *channel,
                                uint16_t command, const uint8_t *payload, uint16_t length);

#ifdef __cplusplus
}
#endif

#endif
