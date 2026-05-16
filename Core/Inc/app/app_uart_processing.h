#ifndef APP_UART_PROCESSING_H
#define APP_UART_PROCESSING_H

#include "app/controllers/ble_controller.h"
#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  float command_1_values[3];
  float command_2_values[2];
  float command_3_value;
  uint32_t command_4_first;
  uint32_t command_4_second;
  uint8_t command_1[6];
  uint8_t command_2[4];
  uint8_t command_3[4];
  uint8_t command_4[8];
  uint8_t command_9[6];
  uint8_t command_130[16];
  uint8_t command_131[16];
  uint16_t last_command;
  uint8_t pc2_inverted_state;
} MotorProtocolState;

void AppUartProcessing_Init(BleController *ble_controller);
void AppUartProcessing_Poll(void);
void AppUartProcessing_OnRxComplete(UART_HandleTypeDef *huart);
void AppUartProcessing_OnTxComplete(UART_HandleTypeDef *huart);
HAL_StatusTypeDef AppUartProcessing_SendMotorResponse(uint16_t code,
                                                      const uint8_t *payload,
                                                      uint16_t length);
const MotorProtocolState *AppUartProcessing_GetMotorState(void);

#ifdef __cplusplus
}
#endif

#endif
