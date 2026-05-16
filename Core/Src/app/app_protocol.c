#include "app/app_protocol.h"

#include <string.h>

enum
{
  APP_PROTOCOL_STATE_WAIT_SOF = 0,
  APP_PROTOCOL_STATE_TYPE,
  APP_PROTOCOL_STATE_LEN_HI,
  APP_PROTOCOL_STATE_LEN_LO,
  APP_PROTOCOL_STATE_PAYLOAD,
};

static uint8_t AppProtocol_AppendEscaped(uint8_t *frame, uint16_t *len, uint8_t value)
{
  if (*len >= APP_PROTOCOL_MAX_FRAME)
  {
    return 0;
  }

  frame[(*len)++] = value;
  if (value == APP_PROTOCOL_SOF)
  {
    if (*len >= APP_PROTOCOL_MAX_FRAME)
    {
      return 0;
    }
    frame[(*len)++] = 0x00;
  }

  return 1;
}

static HAL_StatusTypeDef AppProtocol_StartTransmit(AppProtocolChannel *channel,
                                                   const uint8_t *frame,
                                                   uint16_t length)
{
  if ((channel == NULL) || (channel->huart == NULL) || (frame == NULL) ||
      (length == 0U) || (length > APP_PROTOCOL_MAX_FRAME))
  {
    return HAL_ERROR;
  }

  memcpy(channel->tx_frame, frame, length);
  channel->tx_busy = 1;
  if (HAL_UART_Transmit_IT(channel->huart, channel->tx_frame, length) != HAL_OK)
  {
    channel->tx_busy = 0;
    return HAL_ERROR;
  }

  return HAL_OK;
}

static HAL_StatusTypeDef AppProtocol_QueueFrame(AppProtocolChannel *channel,
                                                const uint8_t *frame,
                                                uint16_t length)
{
  if ((channel == NULL) || (frame == NULL) || (length == 0U) ||
      (length > APP_PROTOCOL_MAX_FRAME) ||
      (channel->tx_queue_count >= APP_PROTOCOL_TX_QUEUE_DEPTH))
  {
    return HAL_ERROR;
  }

  memcpy(channel->tx_queue[channel->tx_queue_tail], frame, length);
  channel->tx_queue_len[channel->tx_queue_tail] = length;
  channel->tx_queue_tail++;
  if (channel->tx_queue_tail >= APP_PROTOCOL_TX_QUEUE_DEPTH)
  {
    channel->tx_queue_tail = 0;
  }
  channel->tx_queue_count++;

  return HAL_OK;
}

static void AppProtocol_StartNextQueuedFrame(AppProtocolChannel *channel)
{
  uint16_t length;

  if ((channel == NULL) || (channel->tx_queue_count == 0U))
  {
    if (channel != NULL)
    {
      channel->tx_busy = 0;
    }
    return;
  }

  length = channel->tx_queue_len[channel->tx_queue_head];
  memcpy(channel->tx_frame, channel->tx_queue[channel->tx_queue_head], length);
  channel->tx_queue_head++;
  if (channel->tx_queue_head >= APP_PROTOCOL_TX_QUEUE_DEPTH)
  {
    channel->tx_queue_head = 0;
  }
  channel->tx_queue_count--;
  channel->tx_busy = 1;
  if (HAL_UART_Transmit_IT(channel->huart, channel->tx_frame, length) != HAL_OK)
  {
    channel->tx_busy = 0;
  }
}

static const uint8_t s_crc_hi_table[256] =
{
  0x00U, 0xC1U, 0x81U, 0x40U, 0x01U, 0xC0U, 0x80U, 0x41U, 0x01U, 0xC0U, 0x80U, 0x41U, 0x00U, 0xC1U, 0x81U, 0x40U,
  0x01U, 0xC0U, 0x80U, 0x41U, 0x00U, 0xC1U, 0x81U, 0x40U, 0x00U, 0xC1U, 0x81U, 0x40U, 0x01U, 0xC0U, 0x80U, 0x41U,
  0x01U, 0xC0U, 0x80U, 0x41U, 0x00U, 0xC1U, 0x81U, 0x40U, 0x00U, 0xC1U, 0x81U, 0x40U, 0x01U, 0xC0U, 0x80U, 0x41U,
  0x00U, 0xC1U, 0x81U, 0x40U, 0x01U, 0xC0U, 0x80U, 0x41U, 0x01U, 0xC0U, 0x80U, 0x41U, 0x00U, 0xC1U, 0x81U, 0x40U,
  0x01U, 0xC0U, 0x80U, 0x41U, 0x00U, 0xC1U, 0x81U, 0x40U, 0x00U, 0xC1U, 0x81U, 0x40U, 0x01U, 0xC0U, 0x80U, 0x41U,
  0x00U, 0xC1U, 0x81U, 0x40U, 0x01U, 0xC0U, 0x80U, 0x41U, 0x01U, 0xC0U, 0x80U, 0x41U, 0x00U, 0xC1U, 0x81U, 0x40U,
  0x00U, 0xC1U, 0x81U, 0x40U, 0x01U, 0xC0U, 0x80U, 0x41U, 0x01U, 0xC0U, 0x80U, 0x41U, 0x00U, 0xC1U, 0x81U, 0x40U,
  0x01U, 0xC0U, 0x80U, 0x41U, 0x00U, 0xC1U, 0x81U, 0x40U, 0x00U, 0xC1U, 0x81U, 0x40U, 0x01U, 0xC0U, 0x80U, 0x41U,
  0x01U, 0xC0U, 0x80U, 0x41U, 0x00U, 0xC1U, 0x81U, 0x40U, 0x00U, 0xC1U, 0x81U, 0x40U, 0x01U, 0xC0U, 0x80U, 0x41U,
  0x00U, 0xC1U, 0x81U, 0x40U, 0x01U, 0xC0U, 0x80U, 0x41U, 0x01U, 0xC0U, 0x80U, 0x41U, 0x00U, 0xC1U, 0x81U, 0x40U,
  0x00U, 0xC1U, 0x81U, 0x40U, 0x01U, 0xC0U, 0x80U, 0x41U, 0x01U, 0xC0U, 0x80U, 0x41U, 0x00U, 0xC1U, 0x81U, 0x40U,
  0x01U, 0xC0U, 0x80U, 0x41U, 0x00U, 0xC1U, 0x81U, 0x40U, 0x00U, 0xC1U, 0x81U, 0x40U, 0x01U, 0xC0U, 0x80U, 0x41U,
  0x00U, 0xC1U, 0x81U, 0x40U, 0x01U, 0xC0U, 0x80U, 0x41U, 0x01U, 0xC0U, 0x80U, 0x41U, 0x00U, 0xC1U, 0x81U, 0x40U,
  0x01U, 0xC0U, 0x80U, 0x41U, 0x00U, 0xC1U, 0x81U, 0x40U, 0x00U, 0xC1U, 0x81U, 0x40U, 0x01U, 0xC0U, 0x80U, 0x41U,
  0x01U, 0xC0U, 0x80U, 0x41U, 0x00U, 0xC1U, 0x81U, 0x40U, 0x00U, 0xC1U, 0x81U, 0x40U, 0x01U, 0xC0U, 0x80U, 0x41U,
  0x00U, 0xC1U, 0x81U, 0x40U, 0x01U, 0xC0U, 0x80U, 0x41U, 0x01U, 0xC0U, 0x80U, 0x41U, 0x00U, 0xC1U, 0x81U, 0x40U
};

static const uint8_t s_crc_lo_table[256] =
{
  0x00U, 0xC0U, 0xC1U, 0x01U, 0xC3U, 0x03U, 0x02U, 0xC2U, 0xC6U, 0x06U, 0x07U, 0xC7U, 0x05U, 0xC5U, 0xC4U, 0x04U,
  0xCCU, 0x0CU, 0x0DU, 0xCDU, 0x0FU, 0xCFU, 0xCEU, 0x0EU, 0x0AU, 0xCAU, 0xCBU, 0x0BU, 0xC9U, 0x09U, 0x08U, 0xC8U,
  0xD8U, 0x18U, 0x19U, 0xD9U, 0x1BU, 0xDBU, 0xDAU, 0x1AU, 0x1EU, 0xDEU, 0xDFU, 0x1FU, 0xDDU, 0x1DU, 0x1CU, 0xDCU,
  0x14U, 0xD4U, 0xD5U, 0x15U, 0xD7U, 0x17U, 0x16U, 0xD6U, 0xD2U, 0x12U, 0x13U, 0xD3U, 0x11U, 0xD1U, 0xD0U, 0x10U,
  0xF0U, 0x30U, 0x31U, 0xF1U, 0x33U, 0xF3U, 0xF2U, 0x32U, 0x36U, 0xF6U, 0xF7U, 0x37U, 0xF5U, 0x35U, 0x34U, 0xF4U,
  0x3CU, 0xFCU, 0xFDU, 0x3DU, 0xFFU, 0x3FU, 0x3EU, 0xFEU, 0xFAU, 0x3AU, 0x3BU, 0xFBU, 0x39U, 0xF9U, 0xF8U, 0x38U,
  0x28U, 0xE8U, 0xE9U, 0x29U, 0xEBU, 0x2BU, 0x2AU, 0xEAU, 0xEEU, 0x2EU, 0x2FU, 0xEFU, 0x2DU, 0xEDU, 0xECU, 0x2CU,
  0xE4U, 0x24U, 0x25U, 0xE5U, 0x27U, 0xE7U, 0xE6U, 0x26U, 0x22U, 0xE2U, 0xE3U, 0x23U, 0xE1U, 0x21U, 0x20U, 0xE0U,
  0xA0U, 0x60U, 0x61U, 0xA1U, 0x63U, 0xA3U, 0xA2U, 0x62U, 0x66U, 0xA6U, 0xA7U, 0x67U, 0xA5U, 0x65U, 0x64U, 0xA4U,
  0x6CU, 0xACU, 0xADU, 0x6DU, 0xAFU, 0x6FU, 0x6EU, 0xAEU, 0xAAU, 0x6AU, 0x6BU, 0xABU, 0x69U, 0xA9U, 0xA8U, 0x68U,
  0x78U, 0xB8U, 0xB9U, 0x79U, 0xBBU, 0x7BU, 0x7AU, 0xBAU, 0xBEU, 0x7EU, 0x7FU, 0xBFU, 0x7DU, 0xBDU, 0xBCU, 0x7CU,
  0xB4U, 0x74U, 0x75U, 0xB5U, 0x77U, 0xB7U, 0xB6U, 0x76U, 0x72U, 0xB2U, 0xB3U, 0x73U, 0xB1U, 0x71U, 0x70U, 0xB0U,
  0x50U, 0x90U, 0x91U, 0x51U, 0x93U, 0x53U, 0x52U, 0x92U, 0x96U, 0x56U, 0x57U, 0x97U, 0x55U, 0x95U, 0x94U, 0x54U,
  0x9CU, 0x5CU, 0x5DU, 0x9DU, 0x5FU, 0x9FU, 0x9EU, 0x5EU, 0x5AU, 0x9AU, 0x9BU, 0x5BU, 0x99U, 0x59U, 0x58U, 0x98U,
  0x88U, 0x48U, 0x49U, 0x89U, 0x4BU, 0x8BU, 0x8AU, 0x4AU, 0x4EU, 0x8EU, 0x8FU, 0x4FU, 0x8DU, 0x4DU, 0x4CU, 0x8CU,
  0x44U, 0x84U, 0x85U, 0x45U, 0x87U, 0x47U, 0x46U, 0x86U, 0x82U, 0x42U, 0x43U, 0x83U, 0x41U, 0x81U, 0x80U, 0x40U
};

uint16_t AppProtocol_Crc16(const uint8_t *data, uint16_t length)
{
  uint8_t crc_lo = 0xFFU;
  uint8_t crc_hi = 0xFFU;

  if ((data == NULL) && (length != 0U))
  {
    return 0xFFFFU;
  }

  while (length-- != 0U)
  {
    uint8_t index = (uint8_t)(*data++ ^ crc_lo);
    crc_lo = (uint8_t)(s_crc_hi_table[index] ^ crc_hi);
    crc_hi = s_crc_lo_table[index];
  }

  return (uint16_t)(((uint16_t)crc_hi << 8) | crc_lo);
}

void AppProtocol_InitChannel(AppProtocolChannel *channel, UART_HandleTypeDef *huart,
                             AppProtocolCommandHandler handler, uint8_t crc_enabled)
{
  if (channel == NULL)
  {
    return;
  }

  memset(channel, 0, sizeof(*channel));
  channel->huart = huart;
  channel->handler = handler;
  channel->crc_enabled = crc_enabled != 0U ? 1U : 0U;
}

HAL_StatusTypeDef AppProtocol_StartReceive(AppProtocolChannel *channel)
{
  if ((channel == NULL) || (channel->huart == NULL))
  {
    return HAL_ERROR;
  }

  return HAL_UART_Receive_IT(channel->huart, &channel->rx_byte, 1);
}

void AppProtocol_OnRxByte(AppProtocolChannel *channel)
{
  uint8_t byte;

  if (channel == NULL)
  {
    return;
  }

  byte = channel->rx_byte;
  (void)AppProtocol_StartReceive(channel);

  if (channel->escaping != 0U)
  {
    channel->escaping = 0;
    if (byte == 0x00U)
    {
      byte = APP_PROTOCOL_SOF;
    }
  }
  else if (byte == APP_PROTOCOL_SOF)
  {
    if (channel->state == APP_PROTOCOL_STATE_WAIT_SOF)
    {
      channel->state = APP_PROTOCOL_STATE_TYPE;
      channel->rx_len = 0;
      channel->expected_len = 0;
      return;
    }

    channel->escaping = 1;
    return;
  }

  switch (channel->state)
  {
    case APP_PROTOCOL_STATE_TYPE:
      channel->state = (byte == APP_PROTOCOL_TYPE_ADDRESS) ? APP_PROTOCOL_STATE_LEN_HI
                                                           : APP_PROTOCOL_STATE_WAIT_SOF;
      break;

    case APP_PROTOCOL_STATE_LEN_HI:
      channel->expected_len = ((uint16_t)byte << 8);
      channel->state = APP_PROTOCOL_STATE_LEN_LO;
      break;

    case APP_PROTOCOL_STATE_LEN_LO:
      channel->expected_len |= byte;
      if ((channel->expected_len == 0U) ||
          (channel->expected_len > sizeof(channel->rx_decoded)))
      {
        channel->state = APP_PROTOCOL_STATE_WAIT_SOF;
      }
      else
      {
        channel->rx_len = 0;
        channel->state = APP_PROTOCOL_STATE_PAYLOAD;
      }
      break;

    case APP_PROTOCOL_STATE_PAYLOAD:
      channel->rx_decoded[channel->rx_len++] = byte;
      if (channel->rx_len >= channel->expected_len)
      {
        uint16_t body_len = channel->rx_len;
        uint8_t frame_ok = 1;

        if (channel->crc_enabled != 0U)
        {
          if (body_len < 4U)
          {
            frame_ok = 0;
          }
          else
          {
            uint16_t expected_crc = (uint16_t)(((uint16_t)channel->rx_decoded[body_len - 2U] << 8) |
                                               channel->rx_decoded[body_len - 1U]);
            body_len = (uint16_t)(body_len - 2U);
            if (AppProtocol_Crc16(channel->rx_decoded, body_len) != expected_crc)
            {
              frame_ok = 0;
            }
          }
        }

        if ((frame_ok != 0U) && (channel->handler != NULL) && (body_len >= 2U))
        {
          uint16_t payload_len = (uint16_t)(body_len - 2U);
          uint16_t command = (uint16_t)(((uint16_t)channel->rx_decoded[payload_len] << 8) |
                                        channel->rx_decoded[payload_len + 1U]);
          channel->handler(channel, command, channel->rx_decoded, payload_len);
        }
        channel->state = APP_PROTOCOL_STATE_WAIT_SOF;
        channel->rx_len = 0;
        channel->expected_len = 0;
      }
      break;

    default:
      channel->state = APP_PROTOCOL_STATE_WAIT_SOF;
      break;
  }
}

void AppProtocol_OnTxComplete(AppProtocolChannel *channel)
{
  if (channel != NULL)
  {
    AppProtocol_StartNextQueuedFrame(channel);
  }
}

HAL_StatusTypeDef AppProtocol_SendResponse(AppProtocolChannel *channel, uint16_t code,
                                           const uint8_t *payload, uint16_t length)
{
  uint8_t plain[APP_PROTOCOL_MAX_PAYLOAD + 2U];
  uint8_t frame[APP_PROTOCOL_MAX_FRAME];
  uint16_t plain_len;
  uint16_t wire_len;
  uint16_t frame_len = 0;

  if ((channel == NULL) || (channel->huart == NULL) || (length > APP_PROTOCOL_MAX_PAYLOAD) ||
      ((payload == NULL) && (length != 0U)))
  {
    return HAL_ERROR;
  }

  if (length != 0U)
  {
    memcpy(plain, payload, length);
  }
  plain[length] = (uint8_t)(code >> 8);
  plain[length + 1U] = (uint8_t)code;
  plain_len = (uint16_t)(length + 2U);
  wire_len = (uint16_t)(plain_len + (channel->crc_enabled != 0U ? 2U : 0U));

  frame[frame_len++] = APP_PROTOCOL_SOF;
  if ((AppProtocol_AppendEscaped(frame, &frame_len, APP_PROTOCOL_TYPE_ADDRESS) == 0U) ||
      (AppProtocol_AppendEscaped(frame, &frame_len, (uint8_t)(wire_len >> 8)) == 0U) ||
      (AppProtocol_AppendEscaped(frame, &frame_len, (uint8_t)wire_len) == 0U))
  {
    return HAL_ERROR;
  }

  for (uint16_t i = 0; i < plain_len; i++)
  {
    if (AppProtocol_AppendEscaped(frame, &frame_len, plain[i]) == 0U)
    {
      return HAL_ERROR;
    }
  }

  if (channel->crc_enabled != 0U)
  {
    uint16_t crc = AppProtocol_Crc16(plain, plain_len);
    if ((AppProtocol_AppendEscaped(frame, &frame_len, (uint8_t)(crc >> 8)) == 0U) ||
        (AppProtocol_AppendEscaped(frame, &frame_len, (uint8_t)crc) == 0U))
    {
      return HAL_ERROR;
    }
  }

  if (channel->tx_busy != 0U)
  {
    return AppProtocol_QueueFrame(channel, frame, frame_len);
  }

  return AppProtocol_StartTransmit(channel, frame, frame_len);
}
