#ifndef APP_APP_H
#define APP_APP_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

void App_Init(void);
void App_Loop(void);
void App_OnGpioExti(uint16_t gpio_pin);

#ifdef __cplusplus
}
#endif

#endif
