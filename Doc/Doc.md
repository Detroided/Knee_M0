
# Components

## STM32L151RCT
Основной контроллер
## 24Cxx
I2C EEPROM, объем точно распознать не удалось, [datasheet](C:\Users\pc\STM32CubeIDE\workspace_1.8.0\Knee_M0\Doc\S-24C02DI-J8T1U5.PDF)
Подключен к I2C2

## 24FC512
I2C EEPROM, объем точно распознать не удалось, [datasheet](C:\Users\pc\STM32CubeIDE\workspace_1.8.0\Knee_M0\Doc\24FC512.pdf)
Подключен к I2C2

## SPI ADC
Похожа на ADS1118IDGS [datasheet](C:\Users\pc\STM32CubeIDE\workspace_1.8.0\Knee_M0\Doc\ADS1118IDGS.pdf)
Подключен к SPI1

## KMT32B
Датчик магнитного поля [datasheet](C:\Users\pc\STM32CubeIDE\workspace_1.8.0\Knee_M0\Doc\ENG_DS_KMT32B_A1.pdf)
Подключен к SPI ADC

## MPU-6050
Инерциальный датчик, [datasheet](C:\Users\pc\STM32CubeIDE\workspace_1.8.0\Knee_M0\Doc\MPU-6050.pdf)
Подключен к I2C1


# PinOut STM32L151 MainBoardFW.hex

Документ собран по `MX_GPIO_Init`, MSP init-функциям, `App_Main` и reverse notes. Это реконструкция, не оригинальная CubeMX-разметка.

## UART

| Пин  | Периферия     | Функция                         | Назначение |
| ---- | ------------- | ------------------------------- | ---------- |
| PA9  | USART1_TX AF7 | UART1 TX, 115200, DMA1_Channel5 | DEBUG PORT |
| PA10 | USART1_RX AF7 | UART1 RX, 115200, DMA1_Channel4 | DEBUG PORT |
| PA2  | USART2_TX AF7 | UART2 TX, 9600, DMA1_Channel6   | BLE_RX     |
| PA3  | USART2_RX AF7 | UART2 RX, 9600, DMA1_Channel7   | BLE_TX     |
| PC10 | USART3_TX AF7 | UART3 TX, 115200, DMA1_Channel3 | MOTOR_RX   |
| PC11 | USART3_RX AF7 | UART3 RX, 115200, DMA1_Channel2 | MOTOR_TX   |

## I2C

| Пин | Периферия | Функция |
|---|---|---|
| PB6 | I2C1_SCL AF4 | Шина MPU6050; также используется в `I2C1_BusRecovery` как toggled SCL |
| PB7 | I2C1_SDA AF4 | Шина MPU6050 |
| PB10 | I2C2_SCL AF4 | Шина EEPROM 24Cxx и I2C2 sensor HAL address `0x68` |
| PB11 | I2C2_SDA AF4 | Шина EEPROM 24Cxx и I2C2 sensor HAL address `0x68` |

## SPI1

| Пин | Периферия | Функция |
|---|---|---|
| PA4 | GPIO output | Software CS/NSS для SPI device at `0x20000AF0`, вероятно active-low |
| PA5 | SPI1_SCK AF5 | SPI1 clock |
| PA6 | SPI1_MISO AF5 | SPI1 MISO |
| PA7 | SPI1_MOSI AF5 | SPI1 MOSI |

## ADC

| Пин | ADC channel | Функция                                                |                      |
| --- | ----------: | ------------------------------------------------------ | -------------------- |
| PB0 |     ADC_IN8 | Аналоговое измерение: `raw / 4096 * 2 * 3.3`           | ADC_8_U6_POWER_IN_V  |
| PB1 |     ADC_IN9 | Аналоговое измерение: `raw / 4096 * 2 * 3.3`           | ADC_9_U6_POWER_OUT_V |
| PC3 |    ADC_IN13 | Масштабированное измерение: `raw / 4096 * 4.091 * 3.3` | V DC-DC 7.4V         |

## GPIO outputs / controls

| Пин  | Начальный уровень            | Реконструированная функция                                                               |            |
| ---- | ---------------------------- | ---------------------------------------------------------------------------------------- | ---------- |
| PC13 | HIGH                         | Управляет индикацией диода<br>Управляемый inverted output: `GPIO_PC13_WriteInverted`     | LED        |
| PC12 | LOW                          | Управляет пином BLE модуля<br>Управляемый output: `GPIO_PC12_Write`                      |            |
| PC5  | HIGH                         | Управляемый output: `GPIO_PC5_Write`                                                     |            |
| PC4  | LOW                          | Управляемый inverted output: `GPIO_PC4_WriteInverted`, `GPIO_PC4_WriteInvertedIfChanged` |            |
| PC2  | HIGH after init              | Управляет транзитором, который активирует U11                                            | U11_CTR_VT |
| PC1  | HIGH                         | GPIO output, управляет вибромотором                                                      |            |
| PC0  | LOW                          | GPIO output, точная внешняя функция не доказана                                          |            |
| PA1  | LOW                          | GPIO output, управляет отладочным диодом                                                 |            |
| PB12 | HIGH                         | GPIO output, точная внешняя функция не доказана                                          |            |
| PB5  | LOW/HIGH in low-power config | Управляемый inverted output: `GPIO_PB5_WriteInverted`                                    |            |
| PB3  | LOW                          | GPIO output, BLE power EN                                                                | BLE_PWR_EN |
| PD2  | output                       | GPIO output,  reset BLE                                                                  | BLE_RST    |

## GPIO inputs / interrupts

| Пин | Режим                        | Реконструированная функция                                                                                                                             |                    |
| --- | ---------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------ | ------------------ |
| PB4 | input                        | Дискретный вход, наналичия индикатор питания                                                                                                           | INDICATION_PWR_CTR |
| PB8 | EXTI rising/falling, no pull | Прерывание от MPU6050<br>Внешний interrupt/input; есть также `GPIO_PB8_WriteInverted` в safety path, нужна ручная проверка конфигурационного конфликта | MPU6050_INT        |


## Low-power

- `Board_ConfigGPIOForLowPower` перенастраивает GPIOA/GPIOB/GPIOC перед STOP/low-power path.
- `PWR_UltraLowPower_Enable/Disable` управляет `PWR_CR.ULP`.
- `PWR_FastWakeUp_Enable/Disable` управляет `PWR_CR.FWU`.
