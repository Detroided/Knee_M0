#include "app/app_uart_processing.h"

#include "app/app_protocol.h"
#include "main.h"
#include "usart.h"

#include <string.h>

static AppProtocolChannel g_ble_channel;
static AppProtocolChannel g_motor_channel;
static MotorProtocolState g_motor_state;
static BleController *g_ble_controller;

static uint16_t AppUartProcessing_ReadBe16(const uint8_t *payload)
{
  return (uint16_t)(((uint16_t)payload[0] << 8) | payload[1]);
}

static uint32_t AppUartProcessing_ReadLe32(const uint8_t *payload)
{
  return ((uint32_t)payload[0]) |
         ((uint32_t)payload[1] << 8) |
         ((uint32_t)payload[2] << 16) |
         ((uint32_t)payload[3] << 24);
}

static uint32_t AppUartProcessing_ReadBe32(const uint8_t *payload)
{
  return ((uint32_t)payload[0] << 24) |
         ((uint32_t)payload[1] << 16) |
         ((uint32_t)payload[2] << 8) |
         ((uint32_t)payload[3]);
}

static void AppUartProcessing_WritePc2Inverted(uint8_t enabled)
{
  HAL_GPIO_WritePin(U11_CTR_VT_GPIO_Port, U11_CTR_VT_Pin,
                    enabled != 0U ? GPIO_PIN_RESET : GPIO_PIN_SET);
  g_motor_state.pc2_inverted_state = enabled;
}

static void AppUartProcessing_CopyCommand(uint8_t *dst, uint16_t dst_len,
                                          const uint8_t *payload, uint16_t length)
{
  uint16_t copy_len = length < dst_len ? length : dst_len;
  memset(dst, 0, dst_len);
  if ((payload != NULL) && (copy_len != 0U))
  {
    memcpy(dst, payload, copy_len);
  }
}

static void AppUartProcessing_HandleMotorCommand(AppProtocolChannel *channel,
                                                 uint16_t command,
                                                 const uint8_t *payload,
                                                 uint16_t length)
{
  g_motor_state.last_command = command;

  switch (command)
  {
    case 1:
      AppUartProcessing_CopyCommand(g_motor_state.command_1, sizeof(g_motor_state.command_1),
                                    payload, length);
      if (length >= 6U)
      {
        g_motor_state.command_1_values[0] = (float)AppUartProcessing_ReadBe16(&payload[0]) / 1000.0f;
        g_motor_state.command_1_values[1] = (float)AppUartProcessing_ReadBe16(&payload[2]) / 1000.0f;
        g_motor_state.command_1_values[2] = (float)AppUartProcessing_ReadBe16(&payload[4]) / 1000.0f;
      }
      break;
    case 2:
      AppUartProcessing_CopyCommand(g_motor_state.command_2, sizeof(g_motor_state.command_2),
                                    payload, length);
      if (length >= 4U)
      {
        g_motor_state.command_2_values[0] = (float)AppUartProcessing_ReadBe16(&payload[0]) / 1000.0f;
        g_motor_state.command_2_values[1] = (float)AppUartProcessing_ReadBe16(&payload[2]) / 1000.0f;
      }
      break;
    case 3:
      AppUartProcessing_CopyCommand(g_motor_state.command_3, sizeof(g_motor_state.command_3),
                                    payload, length);
      if (length >= 4U)
      {
        uint32_t raw = AppUartProcessing_ReadLe32(payload);
        memcpy(&g_motor_state.command_3_value, &raw, sizeof(g_motor_state.command_3_value));
        if ((g_motor_state.command_3_value <= 65.0f) && (g_motor_state.pc2_inverted_state == 0U))
        {
          AppUartProcessing_WritePc2Inverted(1);
        }
        else if ((g_motor_state.command_3_value > 65.0f) && (g_motor_state.pc2_inverted_state != 0U))
        {
          AppUartProcessing_WritePc2Inverted(0);
        }
      }
      break;
    case 4:
      AppUartProcessing_CopyCommand(g_motor_state.command_4, sizeof(g_motor_state.command_4),
                                    payload, length);
      if (length >= 8U)
      {
        g_motor_state.command_4_first = AppUartProcessing_ReadBe32(&payload[0]);
        g_motor_state.command_4_second = AppUartProcessing_ReadBe32(&payload[4]);
      }
      break;
    case 9:
      AppUartProcessing_CopyCommand(g_motor_state.command_9, sizeof(g_motor_state.command_9),
                                    payload, length);
      break;
    case 0x0130:
      AppUartProcessing_CopyCommand(g_motor_state.command_130, sizeof(g_motor_state.command_130),
                                    payload, length);
      break;
    case 0x0131:
      AppUartProcessing_CopyCommand(g_motor_state.command_131, sizeof(g_motor_state.command_131),
                                    payload, length);
      break;
    default:
      break;
  }

  (void)AppProtocol_SendResponse(channel, command, NULL, 0);
}

static void AppUartProcessing_HandleBleCommand(AppProtocolChannel *channel,
                                               uint16_t command,
                                               const uint8_t *payload,
                                               uint16_t length)
{
  BleController_HandlePacket(g_ble_controller, channel, command, payload, length);
}

void AppUartProcessing_Init(BleController *ble_controller)
{
  memset(&g_motor_state, 0, sizeof(g_motor_state));
  g_ble_controller = ble_controller;

  AppProtocol_InitChannel(&g_ble_channel, &huart2, AppUartProcessing_HandleBleCommand, 1);
  AppProtocol_InitChannel(&g_motor_channel, &huart3, AppUartProcessing_HandleMotorCommand, 1);

  (void)AppProtocol_StartReceive(&g_ble_channel);
  (void)AppProtocol_StartReceive(&g_motor_channel);
}

void AppUartProcessing_Poll(void)
{
}

void AppUartProcessing_OnRxComplete(UART_HandleTypeDef *huart)
{
  if (huart == &huart2)
  {
    AppProtocol_OnRxByte(&g_ble_channel);
  }
  else if (huart == &huart3)
  {
    AppProtocol_OnRxByte(&g_motor_channel);
  }
}

void AppUartProcessing_OnTxComplete(UART_HandleTypeDef *huart)
{
  if (huart == &huart2)
  {
    AppProtocol_OnTxComplete(&g_ble_channel);
  }
  else if (huart == &huart3)
  {
    AppProtocol_OnTxComplete(&g_motor_channel);
  }
}

HAL_StatusTypeDef AppUartProcessing_SendMotorResponse(uint16_t code,
                                                      const uint8_t *payload,
                                                      uint16_t length)
{
  return AppProtocol_SendResponse(&g_motor_channel, code, payload, length);
}

const MotorProtocolState *AppUartProcessing_GetMotorState(void)
{
  return &g_motor_state;
}
