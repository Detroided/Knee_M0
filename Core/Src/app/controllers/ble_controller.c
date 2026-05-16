#include "app/controllers/ble_controller.h"

#include <string.h>

typedef struct
{
  uint16_t command;
  uint16_t param_id;
  uint8_t mode32;
} BleParamCommandMap;

static const BleParamCommandMap g_ble_param_commands[] =
{
  {0x057A, 0x0000, 1},
  {0x012F, 0x0019, 1},
  {0x0130, 0x001A, 1},
  {0x0135, 0x001B, 1},
  {0x0137, 0x001C, 1},
  {0x0138, 0x001F, 1},
  {0x0139, 0x0020, 1},
  {0x013A, 0x0021, 0},
  {0x013B, 0x0022, 1},
  {0x013C, 0x0023, 1},
  {0x013D, 0x0024, 0},
  {0x013E, 0x0027, 1},
  {0x013F, 0x0028, 1},
  {0x0140, 0x0029, 1},
  {0x0141, 0x002A, 1},
  {0x0142, 0x002B, 1},
  {0x0143, 0x002C, 1},
  {0x0144, 0x002D, 1},
  {0x0145, 0x002F, 1},
  {0x0146, 0x0030, 1},
  {0x014E, 0x0015, 0},
};

static const BleParamCommandMap *BleController_FindParamCommand(uint16_t command)
{
  for (uint32_t i = 0; i < (sizeof(g_ble_param_commands) / sizeof(g_ble_param_commands[0])); i++)
  {
    if (g_ble_param_commands[i].command == command)
    {
      return &g_ble_param_commands[i];
    }
  }

  return NULL;
}

static HAL_StatusTypeDef BleController_HandleParam32(BleController *controller,
                                                     AppProtocolChannel *channel,
                                                     const BleParamCommandMap *map,
                                                     const uint8_t *payload,
                                                     uint16_t length)
{
  uint8_t response[5];
  HAL_StatusTypeDef status;

  if ((payload == NULL) || (length == 0U))
  {
    return HAL_ERROR;
  }

  response[0] = payload[0];
  if (payload[0] == 0x01U)
  {
    if (length < 5U)
    {
      return HAL_ERROR;
    }

    status = EepromController_WriteParam32(controller->eeprom, map->param_id, &payload[1]);
    if (status != HAL_OK)
    {
      return status;
    }

    memcpy(&response[1], &payload[1], 4);
  }
  else if (payload[0] == 0x00U)
  {
    status = EepromController_ReadParam32(controller->eeprom, map->param_id, &response[1]);
    if (status != HAL_OK)
    {
      return status;
    }
  }
  else
  {
    return HAL_ERROR;
  }

  status = AppProtocol_SendResponse(channel, map->command, response, sizeof(response));
  return status;
}

static HAL_StatusTypeDef BleController_HandleParam8(BleController *controller,
                                                    AppProtocolChannel *channel,
                                                    const BleParamCommandMap *map,
                                                    const uint8_t *payload,
                                                    uint16_t length)
{
  uint8_t response = 0;
  HAL_StatusTypeDef status;

  if ((payload == NULL) || (length == 0U))
  {
    return HAL_ERROR;
  }

  status = EepromController_WriteParam8(controller->eeprom, map->param_id, payload[0]);
  if (status != HAL_OK)
  {
    return status;
  }

  status = EepromController_ReadParam8(controller->eeprom, map->param_id, &response);
  if (status != HAL_OK)
  {
    return status;
  }

  return AppProtocol_SendResponse(channel, map->command, &response, 1);
}

void BleController_Init(BleController *controller, BleDriver *driver)
{
  if (controller == NULL)
  {
    return;
  }

  controller->driver = driver;
  controller->eeprom = NULL;
  controller->last_command = 0;
  controller->last_param_id = 0;
  controller->last_status = HAL_OK;
}

void BleController_AttachEeprom(BleController *controller, EepromController *eeprom)
{
  if (controller == NULL)
  {
    return;
  }

  controller->eeprom = eeprom;
}

void BleController_SetEnabled(BleController *controller, uint8_t enabled)
{
  if ((controller == NULL) || (controller->driver == NULL))
  {
    return;
  }

  BleDriver_SetPower(controller->driver, enabled != 0U ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void BleController_HandlePacket(BleController *controller, AppProtocolChannel *channel,
                                uint16_t command, const uint8_t *payload, uint16_t length)
{
  const BleParamCommandMap *map;

  if ((controller == NULL) || (channel == NULL) || (controller->eeprom == NULL))
  {
    return;
  }

  controller->last_command = command;
  controller->last_status = HAL_ERROR;

  map = BleController_FindParamCommand(command);
  if (map == NULL)
  {
    /* TODO: port remaining UART2_ProtocolCommandHandler_0801269C branches. */
    return;
  }

  controller->last_param_id = map->param_id;
  if (map->mode32 != 0U)
  {
    controller->last_status = BleController_HandleParam32(controller, channel, map, payload, length);
  }
  else
  {
    controller->last_status = BleController_HandleParam8(controller, channel, map, payload, length);
  }
}
