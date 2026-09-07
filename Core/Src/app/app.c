#include "app/app.h"

#include "adc.h"
#include "app/app_debug_interface.h"
#include "app/app_power.h"
#include "app/app_uart_processing.h"
#include "app/controllers/ads_controller.h"
#include "app/controllers/ble_controller.h"
#include "app/controllers/eeprom_controller.h"
#include "app/controllers/imu_controller.h"
#include "app/controllers/motion_phase_controller.h"
#include "app/controllers/mpu_controller.h"
#include "app/drivers/ads1118_driver.h"
#include "app/drivers/ble_driver.h"
#include "app/drivers/eeprom_driver.h"
#include "app/drivers/mpu6050_driver.h"
#include "gpio.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "wwdg.h"

#define APP_IMU_UPDATE_PERIOD_MS 5U

static Mpu6050Driver g_mpu_driver;
static Ads1118Driver g_ads_driver;
static EepromDriver g_eeprom_driver;
static BleDriver g_ble_driver;

static MpuController g_mpu_controller;
static ImuController g_imu_controller;
static MotionPhaseController g_motion_phase_controller;
static AdsController g_ads_controller;
static EepromController g_eeprom_controller;
static BleController g_ble_controller;

static uint32_t g_last_watchdog_refresh_ms;
static uint32_t g_last_imu_update_ms;
static volatile uint8_t g_mpu_interrupt_pending;

static void App_RefreshWatchdog(void)
{
  uint32_t now = HAL_GetTick();
  if ((now - g_last_watchdog_refresh_ms) >= 1U)
  {
    (void)HAL_WWDG_Refresh(&hwwdg);
    g_last_watchdog_refresh_ms = now;
  }
}

static void App_ApplyRuntimeBoardOutputs(void)
{
  /* Ported from Board_ControlOutputs_Init_080118F0. */
  HAL_GPIO_WritePin(BLE_RST_GPIO_Port, BLE_RST_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(BLE_PWR_EN_GPIO_Port, BLE_PWR_EN_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(U11_CTR_VT_GPIO_Port, U11_CTR_VT_Pin, GPIO_PIN_SET);
}

static void App_ReinitializeRuntimeAfterStop(void)
{
  App_ApplyRuntimeBoardOutputs();
  AppUartProcessing_Init(&g_ble_controller);
  AppDebugInterface_Init(&g_imu_controller, &g_ads_controller);
  (void)MpuController_Probe(&g_mpu_controller);
  App_RefreshWatchdog();
  if (g_mpu_controller.present != 0U)
  {
    (void)MpuController_Configure(&g_mpu_controller);
    App_RefreshWatchdog();
  }
  ImuController_ResetFilter(&g_imu_controller);
  MotionPhaseController_Reset(&g_motion_phase_controller);
  g_mpu_interrupt_pending = 0U;
  g_last_imu_update_ms = HAL_GetTick() - APP_IMU_UPDATE_PERIOD_MS;
}

static void App_UpdateMotionPipeline(void)
{
  if (ImuController_Update(&g_imu_controller) == HAL_OK)
  {
    if (AdsController_UpdateAngleEstimate(&g_ads_controller) == HAL_OK)
    {
      MotionPhaseController_SetAuxiliaryAngle(&g_motion_phase_controller,
                                              g_ads_controller.last_angle_deg,
                                              g_ads_controller.last_filtered_angle_deg);
    }
    (void)MotionPhaseController_Update(&g_motion_phase_controller);
  }

  if (ImuController_ConsumeSleepRequest(&g_imu_controller) != 0U)
  {
    AppPower_RequestSleep(APP_SLEEP_REASON_MOTION_SAFETY);
  }
}

void App_Init(void)
{
  g_last_watchdog_refresh_ms = HAL_GetTick();
  g_last_imu_update_ms = g_last_watchdog_refresh_ms - APP_IMU_UPDATE_PERIOD_MS;
  (void)HAL_WWDG_Refresh(&hwwdg);
  App_ApplyRuntimeBoardOutputs();

  Mpu6050Driver_Init(&g_mpu_driver, &hi2c1);
  Ads1118Driver_Init(&g_ads_driver, &hspi1, SPI1_CS_GPIO_Port, SPI1_CS_Pin);
  EepromDriver_Init(&g_eeprom_driver, &hi2c2, EEPROM_24CXX_I2C_ADDRESS);
  BleDriver_Init(&g_ble_driver, &huart2,
                 BLE_PWR_EN_GPIO_Port, BLE_PWR_EN_Pin,
                 BLE_RST_GPIO_Port, BLE_RST_Pin);

  MpuController_Init(&g_mpu_controller, &g_mpu_driver);
  ImuController_Init(&g_imu_controller, &g_mpu_controller);
  AdsController_Init(&g_ads_controller, &g_ads_driver);
  EepromController_Init(&g_eeprom_controller, &g_eeprom_driver);
  BleController_Init(&g_ble_controller, &g_ble_driver);
  BleController_AttachEeprom(&g_ble_controller, &g_eeprom_controller);
  MotionPhaseController_Init(&g_motion_phase_controller, &g_imu_controller,
                             &g_eeprom_controller, AppUartProcessing_SendMotorResponse);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
  HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);
  HAL_NVIC_SetPriority(USART3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART3_IRQn);
  HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);

  (void)MpuController_Probe(&g_mpu_controller);
  App_RefreshWatchdog();
  if (g_mpu_controller.present != 0U)
  {
    (void)MpuController_Configure(&g_mpu_controller);
    App_RefreshWatchdog();
  }

  AppPower_Init();
  AppUartProcessing_Init(&g_ble_controller);
  AppDebugInterface_Init(&g_imu_controller, &g_ads_controller);
}

void App_Loop(void)
{
  uint32_t now;

  App_RefreshWatchdog();
  AppUartProcessing_Poll();
  AppDebugInterface_Poll();

  if (g_mpu_interrupt_pending != 0U)
  {
    g_mpu_interrupt_pending = 0U;
  }

  now = HAL_GetTick();
  if ((now - g_last_imu_update_ms) >= APP_IMU_UPDATE_PERIOD_MS)
  {
    g_last_imu_update_ms = now;
    App_UpdateMotionPipeline();
  }

  if (AppPower_Poll() != 0U)
  {
    App_ReinitializeRuntimeAfterStop();
  }
}

void App_OnGpioExti(uint16_t gpio_pin)
{
  if (gpio_pin == MPU6050_INT_Pin)
  {
    g_mpu_interrupt_pending = 1;
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  App_OnGpioExti(GPIO_Pin);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if ((huart != NULL) && (huart->Instance == USART1))
  {
    AppDebugInterface_OnRxComplete(huart);
  }
  else
  {
    AppUartProcessing_OnRxComplete(huart);
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if ((huart != NULL) && (huart->Instance == USART1))
  {
    AppDebugInterface_OnTxComplete(huart);
  }
  else
  {
    AppUartProcessing_OnTxComplete(huart);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if ((huart != NULL) && (huart->Instance == USART1))
  {
    AppDebugInterface_OnError(huart);
  }
}
