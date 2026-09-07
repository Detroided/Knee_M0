#include "app/app_debug_interface.h"

#include "adc.h"
#include "gpio.h"
#include "usart.h"

#include <math.h>
#include <stdint.h>
#include <string.h>

#define DEBUG_HEADER_SIZE           5U
#define DEBUG_FIXED_FRAME_SIZE      5U
#define DEBUG_VARIABLE_FLAG         0x80U
#define DEBUG_ID_MASK               0x7FU
#define DEBUG_MAX_PAYLOAD_SIZE      256U
#define DEBUG_TX_BUFFER_SIZE        (DEBUG_HEADER_SIZE + DEBUG_MAX_PAYLOAD_SIZE + 1U)
#define DEBUG_STREAM_DEFAULT_MS     100U
#define DEBUG_STREAM_MIN_MS         50U
#define DEBUG_STREAM_MAX_MS         5000U
#define DEBUG_ADC_FULL_SCALE        4096U
#define DEBUG_ADC8_SCALE_MV         6600U
#define DEBUG_ADC9_SCALE_MV         6600U
#define DEBUG_ADC13_SCALE_MV        13500U
#define DEBUG_LED_ON_STATE          GPIO_PIN_RESET
#define DEBUG_ACTIVE_HIGH_ON_STATE  GPIO_PIN_SET

#define DEBUG_CMD_PING              0x01U
#define DEBUG_CMD_STATUS            0x02U
#define DEBUG_CMD_STREAM_CONTROL    0x10U
#define DEBUG_CMD_LED_CONTROL       0x20U
#define DEBUG_CMD_DBGLED_CONTROL    0x21U
#define DEBUG_CMD_VIBRO_CONTROL     0x22U
#define DEBUG_CMD_ACK               0x80U
#define DEBUG_CMD_ERROR             0x81U
#define DEBUG_CMD_STATUS_RESPONSE   0x82U
#define DEBUG_CMD_STREAM_FRAME      0x83U

#define DEBUG_STATUS_OK             0x00U
#define DEBUG_ERR_BAD_CRC           0x01U
#define DEBUG_ERR_BAD_SIZE          0x02U
#define DEBUG_ERR_RX_OVERFLOW       0x03U
#define DEBUG_ERR_BAD_COMMAND       0x04U
#define DEBUG_ERR_BAD_ARGUMENT      0x05U
#define DEBUG_ERR_DMA_START         0x06U

#define DEBUG_ACTION_OFF            0x00U
#define DEBUG_ACTION_ON             0x01U
#define DEBUG_ACTION_TOGGLE         0x02U

typedef enum
{
  DEBUG_RX_HEADER = 0,
  DEBUG_RX_PAYLOAD
} DebugRxState;

typedef struct
{
  ImuController *imu;
  AdsController *ads;

  uint8_t rx_header[DEBUG_HEADER_SIZE];
  uint8_t rx_payload[DEBUG_MAX_PAYLOAD_SIZE + 1U];
  uint16_t rx_payload_size;
  volatile DebugRxState rx_state;
  volatile uint8_t rx_restart_requested;

  volatile uint8_t packet_ready;
  uint8_t pending_id;
  uint8_t pending_command;
  uint8_t pending_variable;
  uint16_t pending_fixed_payload;
  uint16_t pending_payload_size;
  uint8_t pending_payload[DEBUG_MAX_PAYLOAD_SIZE];

  volatile uint8_t rx_error_ready;
  uint8_t rx_error_id;
  uint8_t rx_error_command;
  uint8_t rx_error_code;

  uint8_t tx_buffer[DEBUG_TX_BUFFER_SIZE];
  uint8_t work_payload[DEBUG_MAX_PAYLOAD_SIZE];
  volatile uint8_t tx_busy;

  uint8_t stream_enabled;
  uint16_t stream_period_ms;
  uint32_t last_stream_ms;
} AppDebugInterface;

static AppDebugInterface g_debug;

static const uint8_t g_crc8_table[256] =
{
  0x00U, 0x07U, 0x0EU, 0x09U, 0x1CU, 0x1BU, 0x12U, 0x15U, 0x38U, 0x3FU, 0x36U, 0x31U, 0x24U, 0x23U, 0x2AU, 0x2DU,
  0x70U, 0x77U, 0x7EU, 0x79U, 0x6CU, 0x6BU, 0x62U, 0x65U, 0x48U, 0x4FU, 0x46U, 0x41U, 0x54U, 0x53U, 0x5AU, 0x5DU,
  0xE0U, 0xE7U, 0xEEU, 0xE9U, 0xFCU, 0xFBU, 0xF2U, 0xF5U, 0xD8U, 0xDFU, 0xD6U, 0xD1U, 0xC4U, 0xC3U, 0xCAU, 0xCDU,
  0x90U, 0x97U, 0x9EU, 0x99U, 0x8CU, 0x8BU, 0x82U, 0x85U, 0xA8U, 0xAFU, 0xA6U, 0xA1U, 0xB4U, 0xB3U, 0xBAU, 0xBDU,
  0xC7U, 0xC0U, 0xC9U, 0xCEU, 0xDBU, 0xDCU, 0xD5U, 0xD2U, 0xFFU, 0xF8U, 0xF1U, 0xF6U, 0xE3U, 0xE4U, 0xEDU, 0xEAU,
  0xB7U, 0xB0U, 0xB9U, 0xBEU, 0xABU, 0xACU, 0xA5U, 0xA2U, 0x8FU, 0x88U, 0x81U, 0x86U, 0x93U, 0x94U, 0x9DU, 0x9AU,
  0x27U, 0x20U, 0x29U, 0x2EU, 0x3BU, 0x3CU, 0x35U, 0x32U, 0x1FU, 0x18U, 0x11U, 0x16U, 0x03U, 0x04U, 0x0DU, 0x0AU,
  0x57U, 0x50U, 0x59U, 0x5EU, 0x4BU, 0x4CU, 0x45U, 0x42U, 0x6FU, 0x68U, 0x61U, 0x66U, 0x73U, 0x74U, 0x7DU, 0x7AU,
  0x89U, 0x8EU, 0x87U, 0x80U, 0x95U, 0x92U, 0x9BU, 0x9CU, 0xB1U, 0xB6U, 0xBFU, 0xB8U, 0xADU, 0xAAU, 0xA3U, 0xA4U,
  0xF9U, 0xFEU, 0xF7U, 0xF0U, 0xE5U, 0xE2U, 0xEBU, 0xECU, 0xC1U, 0xC6U, 0xCFU, 0xC8U, 0xDDU, 0xDAU, 0xD3U, 0xD4U,
  0x69U, 0x6EU, 0x67U, 0x60U, 0x75U, 0x72U, 0x7BU, 0x7CU, 0x51U, 0x56U, 0x5FU, 0x58U, 0x4DU, 0x4AU, 0x43U, 0x44U,
  0x19U, 0x1EU, 0x17U, 0x10U, 0x05U, 0x02U, 0x0BU, 0x0CU, 0x21U, 0x26U, 0x2FU, 0x28U, 0x3DU, 0x3AU, 0x33U, 0x34U,
  0x4EU, 0x49U, 0x40U, 0x47U, 0x52U, 0x55U, 0x5CU, 0x5BU, 0x76U, 0x71U, 0x78U, 0x7FU, 0x6AU, 0x6DU, 0x64U, 0x63U,
  0x3EU, 0x39U, 0x30U, 0x37U, 0x22U, 0x25U, 0x2CU, 0x2BU, 0x06U, 0x01U, 0x08U, 0x0FU, 0x1AU, 0x1DU, 0x14U, 0x13U,
  0xAEU, 0xA9U, 0xA0U, 0xA7U, 0xB2U, 0xB5U, 0xBCU, 0xBBU, 0x96U, 0x91U, 0x98U, 0x9FU, 0x8AU, 0x8DU, 0x84U, 0x83U,
  0xDEU, 0xD9U, 0xD0U, 0xD7U, 0xC2U, 0xC5U, 0xCCU, 0xCBU, 0xE6U, 0xE1U, 0xE8U, 0xEFU, 0xFAU, 0xFDU, 0xF4U, 0xF3U
};

static uint8_t Debug_Crc8(const uint8_t *data, uint16_t size)
{
  uint8_t crc = 0U;

  while (size > 0U)
  {
    crc = g_crc8_table[crc ^ *data];
    data++;
    size--;
  }

  return crc;
}

static uint16_t Debug_LoadU16(const uint8_t *data)
{
  return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

static void Debug_PutU8(uint8_t *payload, uint16_t *offset, uint8_t value)
{
  if (*offset < DEBUG_MAX_PAYLOAD_SIZE)
  {
    payload[*offset] = value;
    (*offset)++;
  }
}

static void Debug_PutU16(uint8_t *payload, uint16_t *offset, uint16_t value)
{
  if ((*offset + 2U) <= DEBUG_MAX_PAYLOAD_SIZE)
  {
    payload[*offset] = (uint8_t)(value & 0xFFU);
    payload[*offset + 1U] = (uint8_t)(value >> 8);
    *offset = (uint16_t)(*offset + 2U);
  }
}

static void Debug_PutU32(uint8_t *payload, uint16_t *offset, uint32_t value)
{
  if ((*offset + 4U) <= DEBUG_MAX_PAYLOAD_SIZE)
  {
    payload[*offset] = (uint8_t)(value & 0xFFU);
    payload[*offset + 1U] = (uint8_t)((value >> 8) & 0xFFU);
    payload[*offset + 2U] = (uint8_t)((value >> 16) & 0xFFU);
    payload[*offset + 3U] = (uint8_t)((value >> 24) & 0xFFU);
    *offset = (uint16_t)(*offset + 4U);
  }
}

static void Debug_PutI32(uint8_t *payload, uint16_t *offset, int32_t value)
{
  Debug_PutU32(payload, offset, (uint32_t)value);
}

static int32_t Debug_ScaleFloat(float value, float scale)
{
  float scaled;

  if (!isfinite(value))
  {
    return 0;
  }

  scaled = value * scale;
  if (scaled > 2147483647.0f)
  {
    return INT32_MAX;
  }
  if (scaled < -2147483648.0f)
  {
    return INT32_MIN;
  }

  return (int32_t)(scaled + ((scaled >= 0.0f) ? 0.5f : -0.5f));
}

static uint32_t Debug_RawToScaledMillivolts(uint16_t raw, uint32_t full_scale_mv)
{
  return (((uint32_t)raw * full_scale_mv) + (DEBUG_ADC_FULL_SCALE / 2U)) / DEBUG_ADC_FULL_SCALE;
}

static GPIO_PinState Debug_OffState(GPIO_PinState on_state)
{
  return (on_state == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET;
}

static uint8_t Debug_ReadLogicalPin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState on_state)
{
  return (HAL_GPIO_ReadPin(port, pin) == on_state) ? 1U : 0U;
}

static uint8_t Debug_ReadRawPin(GPIO_TypeDef *port, uint16_t pin)
{
  return (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_SET) ? 1U : 0U;
}

static uint16_t Debug_ClampPeriod(uint16_t period_ms)
{
  uint16_t period = period_ms;

  if (period == 0U)
  {
    period = DEBUG_STREAM_DEFAULT_MS;
  }
  if (period < DEBUG_STREAM_MIN_MS)
  {
    period = DEBUG_STREAM_MIN_MS;
  }
  else if (period > DEBUG_STREAM_MAX_MS)
  {
    period = DEBUG_STREAM_MAX_MS;
  }

  return period;
}

static void Debug_SetRxError(uint8_t id, uint8_t command, uint8_t code)
{
  g_debug.rx_error_id = (uint8_t)(id & DEBUG_ID_MASK);
  g_debug.rx_error_command = command;
  g_debug.rx_error_code = code;
  g_debug.rx_error_ready = 1U;
}

static void Debug_StartHeaderReceive(void)
{
  g_debug.rx_state = DEBUG_RX_HEADER;
  g_debug.rx_payload_size = 0U;

  if (HAL_UART_Receive_DMA(&huart1, g_debug.rx_header, DEBUG_HEADER_SIZE) != HAL_OK)
  {
    Debug_SetRxError(0U, 0U, DEBUG_ERR_DMA_START);
    g_debug.rx_restart_requested = 1U;
  }
}

static void Debug_StartPayloadReceive(uint16_t payload_size)
{
  g_debug.rx_state = DEBUG_RX_PAYLOAD;
  g_debug.rx_payload_size = payload_size;

  if (HAL_UART_Receive_DMA(&huart1, g_debug.rx_payload, (uint16_t)(payload_size + 1U)) != HAL_OK)
  {
    Debug_SetRxError(g_debug.rx_header[0] & DEBUG_ID_MASK, g_debug.rx_header[1], DEBUG_ERR_DMA_START);
    Debug_StartHeaderReceive();
  }
}

static void Debug_StorePendingPacket(uint8_t id,
                                     uint8_t command,
                                     uint8_t variable,
                                     uint16_t fixed_payload,
                                     const uint8_t *payload,
                                     uint16_t payload_size)
{
  if (g_debug.packet_ready != 0U)
  {
    Debug_SetRxError(id, command, DEBUG_ERR_RX_OVERFLOW);
    return;
  }

  g_debug.pending_id = (uint8_t)(id & DEBUG_ID_MASK);
  g_debug.pending_command = command;
  g_debug.pending_variable = variable;
  g_debug.pending_fixed_payload = fixed_payload;
  g_debug.pending_payload_size = payload_size;
  if ((payload != NULL) && (payload_size > 0U))
  {
    memcpy(g_debug.pending_payload, payload, payload_size);
  }
  g_debug.packet_ready = 1U;
}

static HAL_StatusTypeDef Debug_SendFixed(uint8_t id, uint8_t command, uint16_t payload)
{
  if (g_debug.tx_busy != 0U)
  {
    return HAL_BUSY;
  }

  g_debug.tx_buffer[0] = (uint8_t)(id & DEBUG_ID_MASK);
  g_debug.tx_buffer[1] = command;
  g_debug.tx_buffer[2] = (uint8_t)(payload & 0xFFU);
  g_debug.tx_buffer[3] = (uint8_t)(payload >> 8);
  g_debug.tx_buffer[4] = Debug_Crc8(g_debug.tx_buffer, 4U);

  g_debug.tx_busy = 1U;
  if (HAL_UART_Transmit_DMA(&huart1, g_debug.tx_buffer, DEBUG_FIXED_FRAME_SIZE) != HAL_OK)
  {
    g_debug.tx_busy = 0U;
    return HAL_ERROR;
  }

  return HAL_OK;
}

static HAL_StatusTypeDef Debug_SendVariable(uint8_t id,
                                            uint8_t command,
                                            const uint8_t *payload,
                                            uint16_t payload_size)
{
  if ((g_debug.tx_busy != 0U) || (payload_size > DEBUG_MAX_PAYLOAD_SIZE))
  {
    return HAL_BUSY;
  }
  if ((payload == NULL) && (payload_size > 0U))
  {
    return HAL_ERROR;
  }

  g_debug.tx_buffer[0] = (uint8_t)(DEBUG_VARIABLE_FLAG | (id & DEBUG_ID_MASK));
  g_debug.tx_buffer[1] = command;
  g_debug.tx_buffer[2] = (uint8_t)(payload_size & 0xFFU);
  g_debug.tx_buffer[3] = (uint8_t)(payload_size >> 8);
  g_debug.tx_buffer[4] = Debug_Crc8(g_debug.tx_buffer, 4U);
  if (payload_size > 0U)
  {
    memcpy(&g_debug.tx_buffer[DEBUG_HEADER_SIZE], payload, payload_size);
  }
  g_debug.tx_buffer[DEBUG_HEADER_SIZE + payload_size] = Debug_Crc8(payload, payload_size);

  g_debug.tx_busy = 1U;
  if (HAL_UART_Transmit_DMA(&huart1,
                            g_debug.tx_buffer,
                            (uint16_t)(DEBUG_HEADER_SIZE + payload_size + 1U)) != HAL_OK)
  {
    g_debug.tx_busy = 0U;
    return HAL_ERROR;
  }

  return HAL_OK;
}

static HAL_StatusTypeDef Debug_SendAck(uint8_t id, uint8_t command, uint8_t status)
{
  return Debug_SendFixed(id, DEBUG_CMD_ACK, (uint16_t)status | ((uint16_t)command << 8));
}

static HAL_StatusTypeDef Debug_SendError(uint8_t id, uint8_t command, uint8_t code)
{
  return Debug_SendFixed(id, DEBUG_CMD_ERROR, (uint16_t)code | ((uint16_t)command << 8));
}

static HAL_StatusTypeDef Debug_ReadAdcChannel(uint32_t channel, uint16_t *raw)
{
  ADC_ChannelConfTypeDef config = {0};
  HAL_StatusTypeDef status;

  if (raw == NULL)
  {
    return HAL_ERROR;
  }

  config.Channel = channel;
  config.Rank = ADC_REGULAR_RANK_1;
  config.SamplingTime = ADC_SAMPLETIME_4CYCLES;

  status = HAL_ADC_ConfigChannel(&hadc, &config);
  if (status != HAL_OK)
  {
    return status;
  }

  status = HAL_ADC_Start(&hadc);
  if (status != HAL_OK)
  {
    return status;
  }

  status = HAL_ADC_PollForConversion(&hadc, 2U);
  if (status == HAL_OK)
  {
    *raw = (uint16_t)HAL_ADC_GetValue(&hadc);
  }

  (void)HAL_ADC_Stop(&hadc);
  return status;
}

static void Debug_UpdateAdsSnapshot(void)
{
  if (g_debug.ads == NULL)
  {
    return;
  }

  (void)AdsController_UpdateAngleEstimate(g_debug.ads);
  (void)AdsController_UpdateChannel(g_debug.ads, 2U);
  (void)AdsController_UpdateChannel(g_debug.ads, 3U);
}

static HAL_StatusTypeDef Debug_SendStatusPacket(uint8_t id)
{
  uint8_t *payload = g_debug.work_payload;
  uint16_t offset = 0U;

  Debug_PutU32(payload, &offset, HAL_GetTick());
  Debug_PutU8(payload, &offset, g_debug.stream_enabled);
  Debug_PutU16(payload, &offset, g_debug.stream_period_ms);
  Debug_PutU8(payload, &offset, Debug_ReadLogicalPin(LED_GPIO_Port, LED_Pin, DEBUG_LED_ON_STATE));
  Debug_PutU8(payload, &offset, Debug_ReadRawPin(LED_GPIO_Port, LED_Pin));
  Debug_PutU8(payload, &offset, Debug_ReadLogicalPin(DEBUG_LED1_CTR_GPIO_Port,
                                                     DEBUG_LED1_CTR_Pin,
                                                     DEBUG_ACTIVE_HIGH_ON_STATE));
  Debug_PutU8(payload, &offset, Debug_ReadRawPin(DEBUG_LED1_CTR_GPIO_Port, DEBUG_LED1_CTR_Pin));
  Debug_PutU8(payload, &offset, Debug_ReadLogicalPin(VIBRO_MOTOR_CTR_GPIO_Port,
                                                     VIBRO_MOTOR_CTR_Pin,
                                                     DEBUG_ACTIVE_HIGH_ON_STATE));
  Debug_PutU8(payload, &offset, Debug_ReadRawPin(VIBRO_MOTOR_CTR_GPIO_Port, VIBRO_MOTOR_CTR_Pin));

  return Debug_SendVariable(id, DEBUG_CMD_STATUS_RESPONSE, payload, offset);
}

static HAL_StatusTypeDef Debug_ApplyOutputAction(uint8_t id,
                                                 uint8_t command,
                                                 uint8_t action,
                                                 GPIO_TypeDef *port,
                                                 uint16_t pin,
                                                 GPIO_PinState on_state)
{
  if (action == DEBUG_ACTION_ON)
  {
    HAL_GPIO_WritePin(port, pin, on_state);
  }
  else if (action == DEBUG_ACTION_OFF)
  {
    HAL_GPIO_WritePin(port, pin, Debug_OffState(on_state));
  }
  else if (action == DEBUG_ACTION_TOGGLE)
  {
    HAL_GPIO_TogglePin(port, pin);
  }
  else
  {
    return Debug_SendError(id, command, DEBUG_ERR_BAD_ARGUMENT);
  }

  return Debug_SendAck(id, command, DEBUG_STATUS_OK);
}

static HAL_StatusTypeDef Debug_HandleStreamControl(uint8_t id,
                                                   uint8_t command,
                                                   uint8_t variable,
                                                   uint16_t fixed_payload,
                                                   const uint8_t *payload,
                                                   uint16_t payload_size)
{
  uint8_t enable;
  uint16_t period;

  if (variable != 0U)
  {
    if (payload_size < 1U)
    {
      return Debug_SendError(id, command, DEBUG_ERR_BAD_SIZE);
    }

    enable = (payload[0] != 0U) ? 1U : 0U;
    period = (payload_size >= 3U) ? Debug_LoadU16(&payload[1]) : DEBUG_STREAM_DEFAULT_MS;
  }
  else
  {
    enable = ((fixed_payload & 0x8000U) != 0U) ? 1U : 0U;
    period = (uint16_t)(fixed_payload & 0x7FFFU);
  }

  g_debug.stream_period_ms = Debug_ClampPeriod(period);
  g_debug.last_stream_ms = 0U;
  g_debug.stream_enabled = enable;
  return Debug_SendAck(id, command, DEBUG_STATUS_OK);
}

static HAL_StatusTypeDef Debug_ProcessPacket(uint8_t id,
                                             uint8_t command,
                                             uint8_t variable,
                                             uint16_t fixed_payload,
                                             const uint8_t *payload,
                                             uint16_t payload_size)
{
  uint8_t action;

  switch (command)
  {
    case DEBUG_CMD_PING:
      return Debug_SendAck(id, command, DEBUG_STATUS_OK);

    case DEBUG_CMD_STATUS:
      return Debug_SendStatusPacket(id);

    case DEBUG_CMD_STREAM_CONTROL:
      return Debug_HandleStreamControl(id, command, variable, fixed_payload, payload, payload_size);

    case DEBUG_CMD_LED_CONTROL:
      action = (variable != 0U) ? ((payload_size > 0U) ? payload[0] : 0xFFU) : (uint8_t)(fixed_payload & 0xFFU);
      return Debug_ApplyOutputAction(id, command, action, LED_GPIO_Port, LED_Pin, DEBUG_LED_ON_STATE);

    case DEBUG_CMD_DBGLED_CONTROL:
      action = (variable != 0U) ? ((payload_size > 0U) ? payload[0] : 0xFFU) : (uint8_t)(fixed_payload & 0xFFU);
      return Debug_ApplyOutputAction(id,
                                     command,
                                     action,
                                     DEBUG_LED1_CTR_GPIO_Port,
                                     DEBUG_LED1_CTR_Pin,
                                     DEBUG_ACTIVE_HIGH_ON_STATE);

    case DEBUG_CMD_VIBRO_CONTROL:
      action = (variable != 0U) ? ((payload_size > 0U) ? payload[0] : 0xFFU) : (uint8_t)(fixed_payload & 0xFFU);
      return Debug_ApplyOutputAction(id,
                                     command,
                                     action,
                                     VIBRO_MOTOR_CTR_GPIO_Port,
                                     VIBRO_MOTOR_CTR_Pin,
                                     DEBUG_ACTIVE_HIGH_ON_STATE);

    default:
      return Debug_SendError(id, command, DEBUG_ERR_BAD_COMMAND);
  }
}

static HAL_StatusTypeDef Debug_SendStreamFrame(void)
{
  const ImuControllerState *imu = ImuController_GetState(g_debug.imu);
  uint8_t *payload = g_debug.work_payload;
  uint16_t offset = 0U;
  uint16_t adc8_raw = 0U;
  uint16_t adc9_raw = 0U;
  uint16_t adc13_raw = 0U;
  HAL_StatusTypeDef adc8_status;
  HAL_StatusTypeDef adc9_status;
  HAL_StatusTypeDef adc13_status;

  if (g_debug.tx_busy != 0U)
  {
    return HAL_BUSY;
  }

  Debug_UpdateAdsSnapshot();
  adc8_status = Debug_ReadAdcChannel(ADC_CHANNEL_8, &adc8_raw);
  adc9_status = Debug_ReadAdcChannel(ADC_CHANNEL_9, &adc9_raw);
  adc13_status = Debug_ReadAdcChannel(ADC_CHANNEL_13, &adc13_raw);

  Debug_PutU32(payload, &offset, HAL_GetTick());

  if (imu != NULL)
  {
    Debug_PutI32(payload, &offset, Debug_ScaleFloat(imu->q0, 1000000.0f));
    Debug_PutI32(payload, &offset, Debug_ScaleFloat(imu->q1, 1000000.0f));
    Debug_PutI32(payload, &offset, Debug_ScaleFloat(imu->q2, 1000000.0f));
    Debug_PutI32(payload, &offset, Debug_ScaleFloat(imu->q3, 1000000.0f));
    Debug_PutI32(payload, &offset, Debug_ScaleFloat(imu->roll_deg, 1000.0f));
    Debug_PutI32(payload, &offset, Debug_ScaleFloat(imu->pitch_deg, 1000.0f));
    Debug_PutI32(payload, &offset, Debug_ScaleFloat(imu->yaw_deg, 1000.0f));
    Debug_PutI32(payload, &offset, Debug_ScaleFloat(imu->primary_angle_deg, 1000.0f));
    Debug_PutI32(payload, &offset, Debug_ScaleFloat(imu->secondary_angle_deg, 1000.0f));
    Debug_PutI32(payload, &offset, Debug_ScaleFloat(imu->compensated_angle_deg, 1000.0f));
    Debug_PutI32(payload, &offset, Debug_ScaleFloat(imu->primary_rate_dps, 1000.0f));
    Debug_PutI32(payload, &offset, Debug_ScaleFloat(imu->accel_x_mps2, 1000.0f));
    Debug_PutI32(payload, &offset, Debug_ScaleFloat(imu->accel_y_mps2, 1000.0f));
    Debug_PutI32(payload, &offset, Debug_ScaleFloat(imu->accel_z_mps2, 1000.0f));
    Debug_PutI32(payload, &offset, Debug_ScaleFloat(imu->gyro_x_rads, 1000.0f));
    Debug_PutI32(payload, &offset, Debug_ScaleFloat(imu->gyro_y_rads, 1000.0f));
    Debug_PutI32(payload, &offset, Debug_ScaleFloat(imu->gyro_z_rads, 1000.0f));
    Debug_PutU32(payload, &offset, imu->update_count);
  }
  else
  {
    for (uint8_t i = 0U; i < 17U; i++)
    {
      Debug_PutI32(payload, &offset, 0);
    }
    Debug_PutU32(payload, &offset, 0U);
  }

  if (g_debug.ads != NULL)
  {
    Debug_PutU16(payload, &offset, g_debug.ads->last_raw[0]);
    Debug_PutU16(payload, &offset, g_debug.ads->last_raw[1]);
    Debug_PutU16(payload, &offset, g_debug.ads->last_raw[2]);
    Debug_PutU16(payload, &offset, g_debug.ads->last_raw[3]);
    Debug_PutI32(payload, &offset, Debug_ScaleFloat(g_debug.ads->normalized_channel[0], 1000000.0f));
    Debug_PutI32(payload, &offset, Debug_ScaleFloat(g_debug.ads->normalized_channel[1], 1000000.0f));
    Debug_PutI32(payload, &offset, Debug_ScaleFloat(g_debug.ads->last_raw_angle_deg, 1000.0f));
    Debug_PutI32(payload, &offset, Debug_ScaleFloat(g_debug.ads->last_angle_deg, 1000.0f));
    Debug_PutI32(payload, &offset, Debug_ScaleFloat(g_debug.ads->last_filtered_angle_deg, 1000.0f));
    Debug_PutU8(payload, &offset, g_debug.ads->angle_valid);
  }
  else
  {
    for (uint8_t i = 0U; i < 4U; i++)
    {
      Debug_PutU16(payload, &offset, 0U);
    }
    for (uint8_t i = 0U; i < 5U; i++)
    {
      Debug_PutI32(payload, &offset, 0);
    }
    Debug_PutU8(payload, &offset, 0U);
  }

  Debug_PutU16(payload, &offset, adc8_raw);
  Debug_PutU16(payload, &offset, (uint16_t)Debug_RawToScaledMillivolts(adc8_raw, DEBUG_ADC8_SCALE_MV));
  Debug_PutU8(payload, &offset, (adc8_status == HAL_OK) ? 1U : 0U);
  Debug_PutU16(payload, &offset, adc9_raw);
  Debug_PutU16(payload, &offset, (uint16_t)Debug_RawToScaledMillivolts(adc9_raw, DEBUG_ADC9_SCALE_MV));
  Debug_PutU8(payload, &offset, (adc9_status == HAL_OK) ? 1U : 0U);
  Debug_PutU16(payload, &offset, adc13_raw);
  Debug_PutU16(payload, &offset, (uint16_t)Debug_RawToScaledMillivolts(adc13_raw, DEBUG_ADC13_SCALE_MV));
  Debug_PutU8(payload, &offset, (adc13_status == HAL_OK) ? 1U : 0U);

  return Debug_SendVariable(0U, DEBUG_CMD_STREAM_FRAME, payload, offset);
}

void AppDebugInterface_Init(ImuController *imu, AdsController *ads)
{
  g_debug.imu = imu;
  g_debug.ads = ads;
  g_debug.packet_ready = 0U;
  g_debug.rx_error_ready = 0U;
  g_debug.rx_restart_requested = 0U;
  g_debug.tx_busy = 0U;
  g_debug.stream_enabled = 0U;
  g_debug.stream_period_ms = DEBUG_STREAM_DEFAULT_MS;
  g_debug.last_stream_ms = 0U;

  (void)HAL_UART_AbortReceive(&huart1);
  (void)HAL_UART_AbortTransmit(&huart1);
  Debug_StartHeaderReceive();
}

void AppDebugInterface_Poll(void)
{
  uint8_t id;
  uint8_t command;
  uint8_t variable;
  uint16_t fixed_payload;
  uint16_t payload_size;
  uint8_t *payload = g_debug.work_payload;
  uint8_t error_ready;
  uint8_t error_id;
  uint8_t error_command;
  uint8_t error_code;
  uint32_t now;

  if (g_debug.rx_restart_requested != 0U)
  {
    g_debug.rx_restart_requested = 0U;
    (void)HAL_UART_AbortReceive(&huart1);
    Debug_StartHeaderReceive();
  }

  if ((g_debug.rx_error_ready != 0U) && (g_debug.tx_busy == 0U))
  {
    __disable_irq();
    error_ready = g_debug.rx_error_ready;
    error_id = g_debug.rx_error_id;
    error_command = g_debug.rx_error_command;
    error_code = g_debug.rx_error_code;
    g_debug.rx_error_ready = 0U;
    __enable_irq();

    if (error_ready != 0U)
    {
      (void)Debug_SendError(error_id, error_command, error_code);
      return;
    }
  }

  if ((g_debug.packet_ready != 0U) && (g_debug.tx_busy == 0U))
  {
    __disable_irq();
    id = g_debug.pending_id;
    command = g_debug.pending_command;
    variable = g_debug.pending_variable;
    fixed_payload = g_debug.pending_fixed_payload;
    payload_size = g_debug.pending_payload_size;
    if (payload_size > 0U)
    {
      memcpy(payload, g_debug.pending_payload, payload_size);
    }
    g_debug.packet_ready = 0U;
    __enable_irq();

    (void)Debug_ProcessPacket(id, command, variable, fixed_payload, payload, payload_size);
    return;
  }

  if ((g_debug.stream_enabled != 0U) && (g_debug.tx_busy == 0U))
  {
    now = HAL_GetTick();
    if ((g_debug.last_stream_ms == 0U) || ((now - g_debug.last_stream_ms) >= g_debug.stream_period_ms))
    {
      g_debug.last_stream_ms = now;
      (void)Debug_SendStreamFrame();
    }
  }
}

void AppDebugInterface_OnRxComplete(UART_HandleTypeDef *huart)
{
  uint8_t frame_id;
  uint8_t command;
  uint16_t payload_size;
  uint16_t fixed_payload;

  if ((huart == NULL) || (huart->Instance != USART1))
  {
    return;
  }

  if (g_debug.rx_state == DEBUG_RX_HEADER)
  {
    frame_id = (uint8_t)(g_debug.rx_header[0] & DEBUG_ID_MASK);
    command = g_debug.rx_header[1];

    if ((g_debug.rx_header[0] & DEBUG_VARIABLE_FLAG) == 0U)
    {
      if (Debug_Crc8(g_debug.rx_header, 4U) != g_debug.rx_header[4])
      {
        Debug_SetRxError(frame_id, command, DEBUG_ERR_BAD_CRC);
      }
      else
      {
        fixed_payload = Debug_LoadU16(&g_debug.rx_header[2]);
        Debug_StorePendingPacket(frame_id, command, 0U, fixed_payload, NULL, 0U);
      }
      Debug_StartHeaderReceive();
    }
    else
    {
      if (Debug_Crc8(g_debug.rx_header, 4U) != g_debug.rx_header[4])
      {
        Debug_SetRxError(frame_id, command, DEBUG_ERR_BAD_CRC);
        Debug_StartHeaderReceive();
        return;
      }

      payload_size = Debug_LoadU16(&g_debug.rx_header[2]);
      if (payload_size > DEBUG_MAX_PAYLOAD_SIZE)
      {
        Debug_SetRxError(frame_id, command, DEBUG_ERR_BAD_SIZE);
        Debug_StartHeaderReceive();
        return;
      }

      Debug_StartPayloadReceive(payload_size);
    }
  }
  else
  {
    frame_id = (uint8_t)(g_debug.rx_header[0] & DEBUG_ID_MASK);
    command = g_debug.rx_header[1];
    payload_size = g_debug.rx_payload_size;
    if (Debug_Crc8(g_debug.rx_payload, payload_size) != g_debug.rx_payload[payload_size])
    {
      Debug_SetRxError(frame_id, command, DEBUG_ERR_BAD_CRC);
    }
    else
    {
      Debug_StorePendingPacket(frame_id,
                               command,
                               1U,
                               0U,
                               g_debug.rx_payload,
                               payload_size);
    }
    Debug_StartHeaderReceive();
  }
}

void AppDebugInterface_OnTxComplete(UART_HandleTypeDef *huart)
{
  if ((huart != NULL) && (huart->Instance == USART1))
  {
    g_debug.tx_busy = 0U;
  }
}

void AppDebugInterface_OnError(UART_HandleTypeDef *huart)
{
  if ((huart != NULL) && (huart->Instance == USART1))
  {
    g_debug.tx_busy = 0U;
    Debug_SetRxError(0U, 0U, DEBUG_ERR_DMA_START);
    g_debug.rx_restart_requested = 1U;
  }
}
