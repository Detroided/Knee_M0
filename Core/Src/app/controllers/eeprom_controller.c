#include "app/controllers/eeprom_controller.h"

void EepromController_Init(EepromController *controller, EepromDriver *driver)
{
  if (controller == NULL)
  {
    return;
  }

  controller->driver = driver;
  controller->ready_status = HAL_ERROR;
  controller->marker_u = 0;
  controller->marker_f = 0;
  controller->marker_w = 0;
}

HAL_StatusTypeDef EepromController_Probe(EepromController *controller)
{
  if ((controller == NULL) || (controller->driver == NULL))
  {
    return HAL_ERROR;
  }

  controller->ready_status = EepromDriver_IsReady(controller->driver);
  return controller->ready_status;
}

HAL_StatusTypeDef EepromController_ReadMarkers(EepromController *controller)
{
  if ((controller == NULL) || (controller->driver == NULL))
  {
    return HAL_ERROR;
  }

  HAL_StatusTypeDef status = EepromDriver_Read(controller->driver, EEPROM_MARKER_U_ADDRESS,
                                               &controller->marker_u, 1);
  if (status != HAL_OK)
  {
    return status;
  }

  status = EepromDriver_Read(controller->driver, EEPROM_MARKER_F_ADDRESS, &controller->marker_f, 1);
  if (status != HAL_OK)
  {
    return status;
  }

  return EepromDriver_Read(controller->driver, EEPROM_MARKER_W_ADDRESS, &controller->marker_w, 1);
}

HAL_StatusTypeDef EepromController_ReadParam8(EepromController *controller,
                                              uint16_t param_id, uint8_t *value)
{
  if ((controller == NULL) || (controller->driver == NULL) || (value == NULL))
  {
    return HAL_ERROR;
  }

  return EepromDriver_Read(controller->driver,
                           (uint16_t)(EEPROM_PARAM_TABLE_BASE + (param_id * 4U)),
                           value, 1);
}

HAL_StatusTypeDef EepromController_WriteParam8(EepromController *controller,
                                               uint16_t param_id, uint8_t value)
{
  if ((controller == NULL) || (controller->driver == NULL))
  {
    return HAL_ERROR;
  }

  return EepromDriver_Write(controller->driver,
                            (uint16_t)(EEPROM_PARAM_TABLE_BASE + (param_id * 4U)),
                            &value, 1);
}

HAL_StatusTypeDef EepromController_ReadParam32(EepromController *controller,
                                               uint16_t param_id, uint8_t value[4])
{
  if ((controller == NULL) || (controller->driver == NULL) || (value == NULL))
  {
    return HAL_ERROR;
  }

  return EepromDriver_Read(controller->driver,
                           (uint16_t)(EEPROM_PARAM_TABLE_BASE + (param_id * 4U)),
                           value, 4);
}

HAL_StatusTypeDef EepromController_WriteParam32(EepromController *controller,
                                                uint16_t param_id, const uint8_t value[4])
{
  if ((controller == NULL) || (controller->driver == NULL) || (value == NULL))
  {
    return HAL_ERROR;
  }

  return EepromDriver_Write(controller->driver,
                            (uint16_t)(EEPROM_PARAM_TABLE_BASE + (param_id * 4U)),
                            value, 4);
}
