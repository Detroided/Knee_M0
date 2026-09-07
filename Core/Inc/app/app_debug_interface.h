#ifndef APP_DEBUG_INTERFACE_H
#define APP_DEBUG_INTERFACE_H

#include "app/controllers/ads_controller.h"
#include "app/controllers/imu_controller.h"
#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

void AppDebugInterface_Init(ImuController *imu, AdsController *ads);
void AppDebugInterface_Poll(void);
void AppDebugInterface_OnRxComplete(UART_HandleTypeDef *huart);
void AppDebugInterface_OnTxComplete(UART_HandleTypeDef *huart);
void AppDebugInterface_OnError(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif
