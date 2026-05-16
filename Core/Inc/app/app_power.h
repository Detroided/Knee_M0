#ifndef APP_POWER_H
#define APP_POWER_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
  APP_SLEEP_REASON_NONE = 0,
  APP_SLEEP_REASON_MOTION_SAFETY = 1,
} AppSleepReason;

void AppPower_Init(void);
void AppPower_RequestSleep(AppSleepReason reason);
uint8_t AppPower_Poll(void);
AppSleepReason AppPower_GetLastReason(void);

#ifdef __cplusplus
}
#endif

#endif
