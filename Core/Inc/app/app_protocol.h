#ifndef APP_PROTOCOL_H
#define APP_PROTOCOL_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

#define APP_PROTOCOL_SOF          ((uint8_t)0x12)
#define APP_PROTOCOL_TYPE_ADDRESS ((uint8_t)0x34)
#define APP_PROTOCOL_MAX_PAYLOAD  ((uint16_t)0x7E)
#define APP_PROTOCOL_MAX_FRAME    ((uint16_t)0x7F)
#define APP_PROTOCOL_TX_QUEUE_DEPTH ((uint8_t)4U)

typedef struct AppProtocolChannel AppProtocolChannel;

typedef void (*AppProtocolCommandHandler)(AppProtocolChannel *channel,
                                          uint16_t command,
                                          const uint8_t *payload,
                                          uint16_t length);

struct AppProtocolChannel
{
  UART_HandleTypeDef *huart;
  AppProtocolCommandHandler handler;
  uint8_t crc_enabled;
  uint8_t rx_byte;
  uint8_t rx_decoded[APP_PROTOCOL_MAX_PAYLOAD + 4U];
  uint16_t rx_len;
  uint16_t expected_len;
  uint8_t state;
  uint8_t escaping;
  uint8_t tx_frame[APP_PROTOCOL_MAX_FRAME];
  uint8_t tx_queue[APP_PROTOCOL_TX_QUEUE_DEPTH][APP_PROTOCOL_MAX_FRAME];
  uint16_t tx_queue_len[APP_PROTOCOL_TX_QUEUE_DEPTH];
  uint8_t tx_queue_head;
  uint8_t tx_queue_tail;
  uint8_t tx_queue_count;
  volatile uint8_t tx_busy;
};

void AppProtocol_InitChannel(AppProtocolChannel *channel, UART_HandleTypeDef *huart,
                             AppProtocolCommandHandler handler, uint8_t crc_enabled);
uint16_t AppProtocol_Crc16(const uint8_t *data, uint16_t length);
HAL_StatusTypeDef AppProtocol_StartReceive(AppProtocolChannel *channel);
void AppProtocol_OnRxByte(AppProtocolChannel *channel);
void AppProtocol_OnTxComplete(AppProtocolChannel *channel);
HAL_StatusTypeDef AppProtocol_SendResponse(AppProtocolChannel *channel, uint16_t code,
                                           const uint8_t *payload, uint16_t length);

#ifdef __cplusplus
}
#endif

#endif
