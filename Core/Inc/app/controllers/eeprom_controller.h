#ifndef APP_CONTROLLERS_EEPROM_CONTROLLER_H
#define APP_CONTROLLERS_EEPROM_CONTROLLER_H

#include "app/drivers/eeprom_driver.h"

#ifdef __cplusplus
extern "C" {
#endif

#define EEPROM_MARKER_U_ADDRESS      ((uint16_t)0x011E)
#define EEPROM_MARKER_F_ADDRESS      ((uint16_t)0x0122)
#define EEPROM_MARKER_W_ADDRESS      ((uint16_t)0x0126)
#define EEPROM_U_BLOCK_ADDRESS       ((uint16_t)0x012E)
#define EEPROM_U_BLOCK_LENGTH        ((uint16_t)0x000B)
#define EEPROM_PARAM_TABLE_BASE      ((uint16_t)10)

typedef struct
{
  EepromDriver *driver;
  HAL_StatusTypeDef ready_status;
  uint8_t marker_u;
  uint8_t marker_f;
  uint8_t marker_w;
} EepromController;

void EepromController_Init(EepromController *controller, EepromDriver *driver);
HAL_StatusTypeDef EepromController_Probe(EepromController *controller);
HAL_StatusTypeDef EepromController_ReadMarkers(EepromController *controller);
HAL_StatusTypeDef EepromController_ReadParam8(EepromController *controller,
                                              uint16_t param_id, uint8_t *value);
HAL_StatusTypeDef EepromController_WriteParam8(EepromController *controller,
                                               uint16_t param_id, uint8_t value);
HAL_StatusTypeDef EepromController_ReadParam32(EepromController *controller,
                                               uint16_t param_id, uint8_t value[4]);
HAL_StatusTypeDef EepromController_WriteParam32(EepromController *controller,
                                                uint16_t param_id, const uint8_t value[4]);

#ifdef __cplusplus
}
#endif

#endif
