#include "app/drivers/eeprom_driver.h"

void EepromDriver_Init(EepromDriver *driver, I2C_HandleTypeDef *hi2c, uint16_t device_address)
{
  if (driver == NULL)
  {
    return;
  }

  driver->hi2c = hi2c;
  driver->device_address = device_address;
}

HAL_StatusTypeDef EepromDriver_IsReady(EepromDriver *driver)
{
  if ((driver == NULL) || (driver->hi2c == NULL))
  {
    return HAL_ERROR;
  }

  return HAL_I2C_IsDeviceReady(driver->hi2c, driver->device_address, 1, EEPROM_24CXX_IO_TIMEOUT_MS);
}

HAL_StatusTypeDef EepromDriver_Read(EepromDriver *driver, uint16_t address,
                                    uint8_t *data, uint16_t length)
{
  if ((driver == NULL) || (driver->hi2c == NULL) || (data == NULL))
  {
    return HAL_ERROR;
  }

  return HAL_I2C_Mem_Read(driver->hi2c, driver->device_address, address,
                          I2C_MEMADD_SIZE_16BIT, data, length, EEPROM_24CXX_IO_TIMEOUT_MS);
}

HAL_StatusTypeDef EepromDriver_Write(EepromDriver *driver, uint16_t address,
                                     const uint8_t *data, uint16_t length)
{
  uint16_t offset = 0;

  if ((driver == NULL) || (driver->hi2c == NULL) || (data == NULL))
  {
    return HAL_ERROR;
  }

  while (offset < length)
  {
    uint16_t current_address = (uint16_t)(address + offset);
    uint16_t page_remaining = (uint16_t)(EEPROM_24CXX_PAGE_SIZE -
                                        (current_address & (EEPROM_24CXX_PAGE_SIZE - 1U)));
    uint16_t chunk = (uint16_t)(length - offset);
    if (chunk > page_remaining)
    {
      chunk = page_remaining;
    }

    HAL_StatusTypeDef status = HAL_I2C_Mem_Write(driver->hi2c, driver->device_address,
                                                 current_address, I2C_MEMADD_SIZE_16BIT,
                                                 (uint8_t *)&data[offset], chunk,
                                                 EEPROM_24CXX_IO_TIMEOUT_MS);
    if (status != HAL_OK)
    {
      return status;
    }

    HAL_Delay(EEPROM_24CXX_WRITE_DELAY_MS);
    offset = (uint16_t)(offset + chunk);
  }

  return HAL_OK;
}
