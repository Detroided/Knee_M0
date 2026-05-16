#ifndef APP_DRIVERS_EEPROM_DRIVER_H
#define APP_DRIVERS_EEPROM_DRIVER_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

#define EEPROM_24CXX_I2C_ADDRESS       ((uint16_t)0xA0)
#define EEPROM_24CXX_PAGE_SIZE         ((uint16_t)0x80)
#define EEPROM_24CXX_WRITE_DELAY_MS    ((uint32_t)50)
#define EEPROM_24CXX_IO_TIMEOUT_MS     ((uint32_t)25)

typedef struct
{
  I2C_HandleTypeDef *hi2c;
  uint16_t device_address;
} EepromDriver;

void EepromDriver_Init(EepromDriver *driver, I2C_HandleTypeDef *hi2c, uint16_t device_address);
HAL_StatusTypeDef EepromDriver_IsReady(EepromDriver *driver);
HAL_StatusTypeDef EepromDriver_Read(EepromDriver *driver, uint16_t address,
                                    uint8_t *data, uint16_t length);
HAL_StatusTypeDef EepromDriver_Write(EepromDriver *driver, uint16_t address,
                                     const uint8_t *data, uint16_t length);

#ifdef __cplusplus
}
#endif

#endif
