/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include <stdio.h>

#define CHECK_BYTE1 0x23
#define CHECK_BYTE2 0x55
#define CHECK_BYTE3 0x48

#define MCP23S17_OP 0x40

#define MCP23S17_W 0x00
#define MCP23S17_R 0x01

#define MCP23S17_IODIRA 0x00
#define MCP23S17_IPOLA 0x02
#define MCP23S17_GPINTENA 0x04
#define MCP23S17_DEFVALA 0x06
#define MCP23S17_INTCONA 0x08
#define MCP23S17_IOCONA 0x0A
#define MCP23S17_GPPUA 0x0C
#define MCP23S17_INTFA 0x0E
#define MCP23S17_INTCAPA 0x10
#define MCP23S17_GPIOA 0x12
#define MCP23S17_OLATA 0x14

#define MCP23S17_IODIRB 0x01
#define MCP23S17_IPOLB 0x03
#define MCP23S17_GPINTENB 0x05
#define MCP23S17_DEFVALB 0x07
#define MCP23S17_INTCONB 0x09
#define MCP23S17_IOCONB 0x0B
#define MCP23S17_GPPUB 0x0D
#define MCP23S17_INTFB 0x0F
#define MCP23S17_INTCAPB 0x11
#define MCP23S17_GPIOB 0x13
#define MCP23S17_OLATB 0x15

#define MCP23S17_Unimplemented 0
#define MCP23S17_INTPOL 1
#define MCP23S17_ODR 2
#define MCP23S17_HAEN 3
#define MCP23S17_DISSLW 4
#define MCP23S17_SEQOP 5
#define MCP23S17_MIRROR 6
#define MCP23S17_BANK 7

#define MCP23S17_COUNT 12

SPI_HandleTypeDef hspi1;

typedef struct {
  GPIO_TypeDef *gpio;
  uint16_t pin;
} MCP23S17;

MCP23S17 MCP23S17_ARRAY[MCP23S17_COUNT] = {
    {GPIOA, GPIO_PIN_10}, {GPIOA, GPIO_PIN_9}, {GPIOA, GPIO_PIN_8},
    {GPIOC, GPIO_PIN_9},  {GPIOC, GPIO_PIN_8}, {GPIOC, GPIO_PIN_7},
    {GPIOB, GPIO_PIN_10}, {GPIOB, GPIO_PIN_2}, {GPIOB, GPIO_PIN_1},
    {GPIOB, GPIO_PIN_0},  {GPIOC, GPIO_PIN_5}, {GPIOC, GPIO_PIN_4},
};

uint16_t ICtoX[256] = {
    9,   28,  10,  29,  11,  30,  12,  31,  19,  18,  17,  16,  15,  14,  32,
    13,  1,   20,  2,   21,  3,   22,  4,   23,  27,  8,   26,  7,   25,  6,
    24,  5,   41,  60,  42,  61,  43,  62,  44,  63,  51,  50,  49,  48,  47,
    46,  64,  45,  33,  52,  34,  53,  35,  54,  36,  55,  59,  40,  58,  39,
    57,  38,  56,  37,  73,  92,  74,  93,  75,  94,  76,  95,  83,  82,  81,
    80,  79,  78,  96,  77,  65,  84,  66,  85,  67,  86,  68,  87,  91,  72,
    90,  71,  89,  70,  88,  69,  105, 124, 106, 125, 107, 126, 108, 127, 115,
    114, 113, 112, 111, 110, 128, 109, 97,  116, 98,  117, 99,  118, 100, 119,
    123, 104, 122, 103, 121, 102, 120, 101, 137, 156, 138, 157, 139, 158, 140,
    159, 147, 146, 145, 144, 143, 142, 160, 141, 129, 148, 130, 149, 131, 150,
    132, 151, 155, 136, 154, 135, 153, 134, 152, 133, 169, 188, 170, 189, 171,
    190, 172, 191, 179, 178, 177, 176, 175, 174, 192, 173, 161, 180, 162, 181,
    163, 182, 164, 183, 187, 168, 186, 167, 185, 166, 184, 165, 201, 220, 202,
    221, 203, 222, 204, 223, 211, 210, 209, 208, 207, 206, 224, 205, 193, 212,
    194, 213, 195, 214, 196, 215, 219, 200, 218, 199, 217, 198, 216, 197, 233,
    252, 234, 253, 235, 254, 236, 255, 243, 242, 241, 240, 239, 238, 256, 237,
    225, 244, 226, 245, 227, 246, 228, 247, 251, 232, 250, 231, 249, 230, 248,
    229};

uint8_t XtoIC[256] = {
    16,  18,  20,  22,  31,  29,  27,  25,  0,   2,   4,   6,   15,  13,  12,
    11,  10,  9,   8,   17,  19,  21,  23,  30,  28,  26,  24,  1,   3,   5,
    7,   14,  48,  50,  52,  54,  63,  61,  59,  57,  32,  34,  36,  38,  47,
    45,  44,  43,  42,  41,  40,  49,  51,  53,  55,  62,  60,  58,  56,  33,
    35,  37,  39,  46,  80,  82,  84,  86,  95,  93,  91,  89,  64,  66,  68,
    70,  79,  77,  76,  75,  74,  73,  72,  81,  83,  85,  87,  94,  92,  90,
    88,  65,  67,  69,  71,  78,  112, 114, 116, 118, 127, 125, 123, 121, 96,
    98,  100, 102, 111, 109, 108, 107, 106, 105, 104, 113, 115, 117, 119, 126,
    124, 122, 120, 97,  99,  101, 103, 110, 144, 146, 148, 150, 159, 157, 155,
    153, 128, 130, 132, 134, 143, 141, 140, 139, 138, 137, 136, 145, 147, 149,
    151, 158, 156, 154, 152, 129, 131, 133, 135, 142, 176, 178, 180, 182, 191,
    189, 187, 185, 160, 162, 164, 166, 175, 173, 172, 171, 170, 169, 168, 177,
    179, 181, 183, 190, 188, 186, 184, 161, 163, 165, 167, 174, 208, 210, 212,
    214, 223, 221, 219, 217, 192, 194, 196, 198, 207, 205, 204, 203, 202, 201,
    200, 209, 211, 213, 215, 222, 220, 218, 216, 193, 195, 197, 199, 206, 240,
    242, 244, 246, 255, 253, 251, 249, 224, 226, 228, 230, 239, 237, 236, 235,
    234, 233, 232, 241, 243, 245, 247, 254, 252, 250, 248, 225, 227, 229, 231,
    238};

uint8_t str_rx[64];

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);

void MCP23S17_write(SPI_HandleTypeDef *hspi, GPIO_TypeDef *GPIOx,
                    uint16_t GPIO_Pin, uint8_t reg, uint8_t data,
                    uint8_t A0A1A2);

uint8_t MCP23S17_read(SPI_HandleTypeDef *hspi, GPIO_TypeDef *GPIOx,
                      uint16_t GPIO_Pin, uint8_t reg, uint8_t A0A1A2);

void MCP23S17_Init();

int checkStr(uint8_t *str);
uint8_t checkCircuit(uint8_t pin, uint8_t *pins);

int main(void) {
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USB_DEVICE_Init();

  MCP23S17_Init();

  volatile size_t slen = 0;
  uint8_t count = 0;
  uint8_t pins[MCP23S17_COUNT * 16];
  memset(pins, 0, sizeof(pins));
  uint8_t outStr[2 * MCP23S17_COUNT * 16 + 3 - 1];
  memset(outStr, 0, sizeof(pins));
  uint8_t chkStr[3] = {CHECK_BYTE1, CHECK_BYTE2, CHECK_BYTE3};

  while (1) {
    slen = strlen((char *)str_rx);
    if (slen != 0) {
      if (!checkStr(str_rx)) {
        memset(pins, 0, sizeof(pins));
        memset(outStr, 0, sizeof(outStr));

        memcpy(outStr, chkStr, sizeof(chkStr));
        int i, k = 3;
        for (i = 3; i < slen; i++) {
          count = checkCircuit(XtoIC[str_rx[i] - 1], pins);
          memcpy(outStr + k, pins, count);
          k += count;
          if (i + 1 != slen) {
            outStr[k] = ';';
            k++;
          } 
        }

        /*count = checkCircuit(XtoIC[str_rx[3] - 1], pins);
        memcpy(outStr, chkStr, sizeof(chkStr));
        memcpy(outStr + 3, pins, count);*/
        CDC_Transmit_FS(outStr, k);
#ifdef DEBUG
        printf("InStr: %s\n", str_rx);
        printf("OutStr: %s\n", outStr);
#endif
      }
      memset(str_rx, 0, sizeof(str_rx));
      count = 0;
    }
  }
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void) {

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(
      GPIOC, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9,
      GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_10,
                    GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10,
                    GPIO_PIN_RESET);

  /*Configure GPIO pins : PC4 PC5 PC7 PC8
                           PC9 */
  GPIO_InitStruct.Pin =
      GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB10 */
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA9 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

void MCP23S17_write(SPI_HandleTypeDef *hspi, GPIO_TypeDef *GPIOx,
                    uint16_t GPIO_Pin, uint8_t reg, uint8_t data,
                    uint8_t A0A1A2) {
  HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
  uint8_t d[3];
  d[0] = MCP23S17_OP | A0A1A2 | MCP23S17_W;
  d[1] = reg;
  d[2] = data;
  HAL_SPI_Transmit(hspi, d, sizeof(d), HAL_MAX_DELAY);
  HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
}

uint8_t MCP23S17_read(SPI_HandleTypeDef *hspi, GPIO_TypeDef *GPIOx,
                      uint16_t GPIO_Pin, uint8_t reg, uint8_t A0A1A2) {
  HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
  uint8_t d[2];
  d[0] = MCP23S17_OP | A0A1A2 | MCP23S17_R;
  d[1] = reg;
  HAL_SPI_Transmit(hspi, d, sizeof(d), HAL_MAX_DELAY);
  uint8_t data = 0;
  HAL_SPI_Receive(hspi, &data, sizeof(data), HAL_MAX_DELAY);
  HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);

  return data;
}

void MCP23S17_Init() {
  int k;
  for (k = 0; k < MCP23S17_COUNT; k++) {
    MCP23S17_write(&hspi1, MCP23S17_ARRAY[k].gpio, MCP23S17_ARRAY[k].pin,
                   MCP23S17_IOCONA, 0b00100000, 0);
    MCP23S17_write(&hspi1, MCP23S17_ARRAY[k].gpio, MCP23S17_ARRAY[k].pin,
                   MCP23S17_IOCONB, 0b00100000, 0);
    MCP23S17_write(&hspi1, MCP23S17_ARRAY[k].gpio, MCP23S17_ARRAY[k].pin,
                   MCP23S17_IPOLA, 0, 0);
    MCP23S17_write(&hspi1, MCP23S17_ARRAY[k].gpio, MCP23S17_ARRAY[k].pin,
                   MCP23S17_IPOLB, 0, 0);
    MCP23S17_write(&hspi1, MCP23S17_ARRAY[k].gpio, MCP23S17_ARRAY[k].pin,
                   MCP23S17_GPINTENA, 0, 0);
    MCP23S17_write(&hspi1, MCP23S17_ARRAY[k].gpio, MCP23S17_ARRAY[k].pin,
                   MCP23S17_GPINTENB, 0, 0);
    MCP23S17_write(&hspi1, MCP23S17_ARRAY[k].gpio, MCP23S17_ARRAY[k].pin,
                   MCP23S17_GPPUA, 0, 0);
    MCP23S17_write(&hspi1, MCP23S17_ARRAY[k].gpio, MCP23S17_ARRAY[k].pin,
                   MCP23S17_GPPUB, 0, 0);
    MCP23S17_write(&hspi1, MCP23S17_ARRAY[k].gpio, MCP23S17_ARRAY[k].pin,
                   MCP23S17_IODIRA, 0, 0);
    MCP23S17_write(&hspi1, MCP23S17_ARRAY[k].gpio, MCP23S17_ARRAY[k].pin,
                   MCP23S17_IODIRB, 0, 0);
    MCP23S17_write(&hspi1, MCP23S17_ARRAY[k].gpio, MCP23S17_ARRAY[k].pin,
                   MCP23S17_GPIOA, 0, 0);
    MCP23S17_write(&hspi1, MCP23S17_ARRAY[k].gpio, MCP23S17_ARRAY[k].pin,
                   MCP23S17_GPIOB, 0, 0);
    MCP23S17_write(&hspi1, MCP23S17_ARRAY[k].gpio, MCP23S17_ARRAY[k].pin,
                   MCP23S17_IODIRA, 0xFF, 0);
    MCP23S17_write(&hspi1, MCP23S17_ARRAY[k].gpio, MCP23S17_ARRAY[k].pin,
                   MCP23S17_IODIRB, 0xFF, 0);
  }
}

int checkStr(uint8_t *str) {
  if (str[0] != CHECK_BYTE1 && str[1] != CHECK_BYTE2 && str[2] != CHECK_BYTE3)
    return 1;
  return 0;
}

uint8_t checkCircuit(uint8_t pin, uint8_t *pins) {
  int i = pin / 16;
  int j = pin % 16;
  if (j < 7) {
    MCP23S17_write(&hspi1, MCP23S17_ARRAY[i].gpio, MCP23S17_ARRAY[i].pin,
                   MCP23S17_IODIRA, 0xFF & ~(1 << j), 0);
    MCP23S17_write(&hspi1, MCP23S17_ARRAY[i].gpio, MCP23S17_ARRAY[i].pin,
                   MCP23S17_GPIOA, 1 << j, 0);
  } else {
    MCP23S17_write(&hspi1, MCP23S17_ARRAY[i].gpio, MCP23S17_ARRAY[i].pin,
                   MCP23S17_IODIRB, 0xFF & ~(1 << (j - 8)), 0);
    MCP23S17_write(&hspi1, MCP23S17_ARRAY[i].gpio, MCP23S17_ARRAY[i].pin,
                   MCP23S17_GPIOB, 1 << (j - 8), 0);
  }
#ifdef DEBUG
  uint8_t a = 0, b = 0;
  int m;
  printf("All Ports:\n");
  for (m = 0; m < MCP23S17_COUNT; m++) {
    a = MCP23S17_read(&hspi1, MCP23S17_ARRAY[m].gpio, MCP23S17_ARRAY[m].pin,
                      MCP23S17_GPIOA, 0);
    b = MCP23S17_read(&hspi1, MCP23S17_ARRAY[m].gpio, MCP23S17_ARRAY[m].pin,
                      MCP23S17_GPIOB, 0);
    printf("%i %i\n", a, b);
  }
#endif
  int k, pinsCount = 0, l;
  uint8_t buf;
  for (k = 0; k < MCP23S17_COUNT; k++) {
    buf = MCP23S17_read(&hspi1, MCP23S17_ARRAY[k].gpio, MCP23S17_ARRAY[k].pin,
                        MCP23S17_GPIOA, 0);
    l = 0;
    while (buf > 0) {
      if (buf & 0x01) {
        pins[pinsCount] = k * 16 + l;
        pinsCount++;
      }
      l++;
      buf >>= 1;
    }
    buf = MCP23S17_read(&hspi1, MCP23S17_ARRAY[k].gpio, MCP23S17_ARRAY[k].pin,
                        MCP23S17_GPIOB, 0);
    l = 0;
    while (buf > 0) {
      if (buf & 0x01) {
        pins[pinsCount] = k * 16 + l + 8;
        pinsCount++;
      }
      l++;
      buf >>= 1;
    }
  }

  MCP23S17_write(&hspi1, MCP23S17_ARRAY[i].gpio, MCP23S17_ARRAY[i].pin,
                 MCP23S17_GPIOA, 0, 0);
  MCP23S17_write(&hspi1, MCP23S17_ARRAY[i].gpio, MCP23S17_ARRAY[i].pin,
                 MCP23S17_IODIRA, 0xFF, 0);
  MCP23S17_write(&hspi1, MCP23S17_ARRAY[i].gpio, MCP23S17_ARRAY[i].pin,
                 MCP23S17_GPIOB, 0, 0);
  MCP23S17_write(&hspi1, MCP23S17_ARRAY[i].gpio, MCP23S17_ARRAY[i].pin,
                 MCP23S17_IODIRB, 0xFF, 0);

  for (k = 0; k < pinsCount; k++) {
    if (pins[k] == pin) {
      uint8_t temp = pins[0];
      pins[0] = pin;
      pins[k] = temp;
      break;
    }
  }
  for (k = 0; k < pinsCount; k++) {
    pins[k] = ICtoX[pins[k]];
  }

  return pinsCount;
}

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line)
   */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
