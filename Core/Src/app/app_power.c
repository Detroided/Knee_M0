#include "app/app_power.h"

#include "adc.h"
#include "dma.h"
#include "gpio.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"

extern void SystemClock_Config(void);

static volatile uint8_t g_sleep_requested;
static AppSleepReason g_sleep_reason;

void AppPower_Init(void)
{
  g_sleep_requested = 0;
  g_sleep_reason = APP_SLEEP_REASON_NONE;
}

void AppPower_RequestSleep(AppSleepReason reason)
{
  g_sleep_reason = reason;
  g_sleep_requested = 1;
}

AppSleepReason AppPower_GetLastReason(void)
{
  return g_sleep_reason;
}

uint8_t AppPower_Poll(void)
{
  if (g_sleep_requested == 0U)
  {
    return 0;
  }

  g_sleep_requested = 0;

  HAL_SuspendTick();
  HAL_UART_DeInit(&huart1);
  HAL_UART_DeInit(&huart2);
  HAL_UART_DeInit(&huart3);
  HAL_SPI_DeInit(&hspi1);
  HAL_I2C_DeInit(&hi2c1);
  HAL_I2C_DeInit(&hi2c2);
  HAL_ADC_DeInit(&hadc);
  HAL_TIM_Base_Stop(&htim2);

  HAL_PWREx_EnableUltraLowPower();
  HAL_PWREx_EnableFastWakeUp();
  HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
  HAL_PWREx_DisableFastWakeUp();
  HAL_PWREx_DisableUltraLowPower();

  SystemClock_Config();
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_ADC_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  HAL_ResumeTick();
  return 1;
}
