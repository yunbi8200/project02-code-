/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "touch_algo.h"
#include "smartink_protocol.h"
/* USER CODE END Includes */
/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */
/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define USE_DMA   1                  /* 1: ADC+DMA 버스트   0: Polling (CPU 점유율 비교용) */
#define DISCARD   6                    /* 전극 전환 직후 버릴 샘플 (약 20us = 안정화 시간) */
#define TOTAL     (DISCARD + TA_BURST) /* 축당 변환 횟수 22 */
#define CH_XR     0U                   /* PA0 = ADC 채널 0 (Y 측정 때 읽음) */
#define CH_YU     4U                   /* PA4 = ADC 채널 4 (X 측정 때 읽음) */
#define M_ANALOG  0x0U                 /* CRL 4비트 모드 */
#define M_OUT     0x2U                 /* 출력 푸시풀 2MHz */
#define M_FLOAT   0x4U                 /* 플로팅 입력 */
#define M_PULL    0x8U                 /* 풀업/풀다운 입력 (ODR=1이면 풀업) */
/* USER CODE END PD */
/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */
/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;
/* USER CODE BEGIN PV */
/* 보정값: 패널 모서리를 눌러 디버그 출력의 raw 값을 보고 바꿀 것 (MIN/MAX를 바꿔 넣으면 방향 반전) */
static const ta_cfg_t TA_CFG = {
  310, 3790,   /* X: 왼쪽 끝, 오른쪽 끝 */
  3720, 260,   /* Y: 위쪽 끝, 아래쪽 끝 */
  6, 1, 2457   /* 적응형 EMA: 느릴 때 1/64, 빠를 때 1/2, 기준 속도 */
};
static volatile uint16_t bufX[TOTAL], bufY[TOTAL];   /* DMA가 채우는 버퍼 */
static volatile uint8_t  axis;                       /* 0: X 측정 중, 1: Y 측정 중 */
static volatile uint32_t busy_cyc;                   /* 인터럽트에서 쓴 CPU 사이클 */
static volatile uint32_t ticks;                      /* 처리한 측정 횟수 */
static ta_out_t out;
static uint8_t  frame[PKT_MAX_FRAME];
static uint8_t  pkt_div;
static uint8_t  tx_seq;
static volatile uint8_t spi_busy;
/* USER CODE END PV */
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */
/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* ───────── GPIO: 전극 모드 전환 (CRL 레지스터 직접) ───────── */
static void pa_mode(uint32_t pin, uint32_t m)
{
  GPIOA->CRL = (GPIOA->CRL & ~(0xFU << (pin * 4U))) | (m << (pin * 4U));
}
static void pb0_mode(uint32_t m)
{
  GPIOB->CRL = (GPIOB->CRL & ~0xFU) | m;
}
/* X 측정: XR=HIGH, XL=LOW, YU=ADC, YD=입력 */
static void mode_x(void)
{
  GPIOA->BSRR = (1U << 0) | (1U << (1 + 16));
  pa_mode(0, M_OUT); pa_mode(1, M_OUT); pa_mode(4, M_ANALOG); pb0_mode(M_FLOAT);
}
/* Y 측정: YU=HIGH, YD=LOW, XR=ADC, XL=입력 */
static void mode_y(void)
{
  GPIOA->BSRR = (1U << 4);
  GPIOB->BSRR = (1U << 16);
  pa_mode(4, M_OUT); pb0_mode(M_OUT); pa_mode(0, M_ANALOG); pa_mode(1, M_FLOAT);
}
/* 터치 감지: XL=LOW, YU=풀업 입력 -> 누르면 PA4가 LOW (다음 1ms 동안 안정화) */
static void mode_touch(void)
{
  GPIOA->BSRR = (1U << (1 + 16)) | (1U << 4);
  pa_mode(1, M_OUT); pa_mode(0, M_FLOAT); pa_mode(4, M_PULL); pb0_mode(M_FLOAT);
}
/* ───────── ADC1 (+ DMA1 채널1) ───────── */
static void ADC1_Init_Reg(void)
{
  RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_ADCPRE) | RCC_CFGR_ADCPRE_DIV6;  /* ADC 클럭 14MHz 이하 */
  RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
  ADC1->SMPR2 = (3U << (CH_XR * 3U)) | (3U << (CH_YU * 3U));      /* 28.5 사이클 */
  ADC1->CR2 = ADC_CR2_ADON;
  for (volatile int i = 0; i < 1000; i++) {}
  ADC1->CR2 |= ADC_CR2_RSTCAL; while (ADC1->CR2 & ADC_CR2_RSTCAL) {}
  ADC1->CR2 |= ADC_CR2_CAL;    while (ADC1->CR2 & ADC_CR2_CAL) {}
#if USE_DMA
  RCC->AHBENR |= RCC_AHBENR_DMA1EN;
  DMA1_Channel1->CPAR = (uint32_t)&ADC1->DR;
  DMA1_Channel1->CCR  = DMA_CCR_MINC | DMA_CCR_PSIZE_0 | DMA_CCR_MSIZE_0 | DMA_CCR_TCIE;  /* Normal 모드 */
  ADC1->CR2 |= ADC_CR2_DMA;
  NVIC_SetPriority(DMA1_Channel1_IRQn, 1);
  NVIC_EnableIRQ(DMA1_Channel1_IRQn);
#endif
}
#if USE_DMA
/* DMA로 TOTAL개 연속 변환 시작 (CPU는 바로 빠져나감) */
static void start_burst(uint32_t ch, volatile uint16_t *dst)
{
  DMA1_Channel1->CCR &= ~DMA_CCR_EN;
  DMA1_Channel1->CMAR  = (uint32_t)dst;
  DMA1_Channel1->CNDTR = TOTAL;
  DMA1_Channel1->CCR  |= DMA_CCR_EN;
  ADC1->SQR3 = ch;
  ADC1->CR2 |= ADC_CR2_CONT;
  ADC1->CR2 |= ADC_CR2_ADON;            /* ADON을 한 번 더 쓰면 변환 시작 */
}
#else
/* Polling: 샘플마다 변환 끝날 때까지 CPU가 기다림 */
static void poll_burst(uint32_t ch, volatile uint16_t *dst)
{
  ADC1->SQR3 = ch;
  for (int i = 0; i < TOTAL; i++) {
    ADC1->CR2 |= ADC_CR2_ADON;
    while (!(ADC1->SR & ADC_SR_EOC)) {}
    dst[i] = (uint16_t)ADC1->DR;
  }
}
#endif
/* ───────── SPI2 마스터 TX (+ DMA1 채널5): #3으로 좌표 패킷 ─────────
 * PB12 NSS(GPIO 출력)  PB13 SCK  PB15 MOSI  -> #3의 PB12, PB13, PB15
 * 모드 0, 8비트, MSB 먼저, 송신 전용 (#3은 SPI2 슬레이브 수신 전용) */
static void SPI2_Init_Reg(void)
{
  RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
  RCC->AHBENR  |= RCC_AHBENR_DMA1EN;
  GPIOB->BSRR = (1U << 12);                                   /* NSS HIGH = 선택 안 함 */
  GPIOB->CRH  = (GPIOB->CRH & ~((0xFU << 16) | (0xFU << 20) | (0xFU << 28)))
              | (0x2U << 16)                                  /* PB12 NSS : 출력 */
              | (0xBU << 20)                                  /* PB13 SCK : AF 푸시풀 */
              | (0xBU << 28);                                 /* PB15 MOSI: AF 푸시풀 */
  SPI2->CR1 = SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI        /* 마스터, NSS는 GPIO로 직접 */
            | SPI_CR1_BIDIMODE | SPI_CR1_BIDIOE               /* 송신 전용 */
            | (5U << 3);                                      /* PCLK1/64 = 약 500kHz */
  SPI2->CR2 = SPI_CR2_TXDMAEN;
  SPI2->CR1 |= SPI_CR1_SPE;
  DMA1_Channel5->CPAR = (uint32_t)&SPI2->DR;
  DMA1_Channel5->CCR  = DMA_CCR_MINC | DMA_CCR_DIR | DMA_CCR_TCIE;   /* 메모리 -> SPI, 8비트 */
  NVIC_SetPriority(DMA1_Channel5_IRQn, 3);
  NVIC_EnableIRQ(DMA1_Channel5_IRQn);
}
static void spi_send(uint16_t n)
{
  spi_busy = 1;
  GPIOB->BSRR = (1U << (12 + 16));                            /* NSS LOW */
  DMA1_Channel5->CCR  &= ~DMA_CCR_EN;
  DMA1_Channel5->CMAR  = (uint32_t)frame;
  DMA1_Channel5->CNDTR = n;
  DMA1_Channel5->CCR  |= DMA_CCR_EN;
}
/* DMA가 마지막 바이트를 넘기면: 전송이 완전히 끝날 때까지 기다렸다가 NSS HIGH */
void DMA1_Channel5_IRQHandler(void)
{
  DMA1->IFCR = DMA_IFCR_CGIF5;
  while (!(SPI2->SR & SPI_SR_TXE)) {}
  while (SPI2->SR & SPI_SR_BSY) {}
  GPIOB->BSRR = (1U << 12);                                   /* NSS HIGH */
  spi_busy = 0;
}
/* ───────── TIM2: 1ms 측정 주기 ───────── */
static void TIM2_Init_Reg(void)
{
  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
  TIM2->PSC  = SystemCoreClock / 1000000U - 1U;   /* 1MHz 카운트 */
  TIM2->ARR  = 1000U - 1U;                        /* 1ms */
  TIM2->DIER = TIM_DIER_UIE;
  TIM2->CR1  = TIM_CR1_CEN;
  NVIC_SetPriority(TIM2_IRQn, 2);
  NVIC_EnableIRQ(TIM2_IRQn);
}
/* DWT 사이클 카운터 (CPU 점유율 측정용) */
static void DWT_Init(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
}
/* 측정 1회 처리: 좌표 계산 + 10회마다 패킷 송신 */
static void process(uint8_t touched)
{
  ta_step(touched, (const uint16_t *)&bufX[DISCARD], (const uint16_t *)&bufY[DISCARD], &out);
  ticks++;
  if (++pkt_div >= TA_FS / 100) {                 /* 100Hz */
    pkt_div = 0;
    if (!spi_busy) {                              /* 이전 패킷 송신 끝났을 때만 */
      touch_data_t t = { tx_seq++, out.x, out.y, out.touch };
      spi_send(pkt_build_touch(&t, frame));
    }
  }
}
void TIM2_IRQHandler(void)
{
  uint32_t t0 = DWT->CYCCNT;
  TIM2->SR = ~TIM_SR_UIF;
  uint8_t touched = (GPIOA->IDR & (1U << 4)) == 0U;   /* 풀업이 LOW로 당겨졌으면 눌림 */
#if USE_DMA
  if (touched) { axis = 0; mode_x(); start_burst(CH_YU, bufX); }   /* 나머지는 DMA 인터럽트에서 */
  else process(0);
#else
  if (touched) {
    mode_x(); poll_burst(CH_YU, bufX);
    mode_y(); poll_burst(CH_XR, bufY);
    mode_touch();
  }
  process(touched);
#endif
  busy_cyc += DWT->CYCCNT - t0;
}
#if USE_DMA
/* 버스트 완료: X가 끝났으면 Y 시작, Y가 끝났으면 계산 */
void DMA1_Channel1_IRQHandler(void)
{
  uint32_t t0 = DWT->CYCCNT;
  DMA1->IFCR = DMA_IFCR_CGIF1;
  ADC1->CR2 &= ~ADC_CR2_CONT;                   /* 연속 변환 멈춤 */
  if (axis == 0) { axis = 1; mode_y(); start_burst(CH_XR, bufY); }
  else           { mode_touch(); process(1); }
  busy_cyc += DWT->CYCCNT - t0;
}
#endif
/* USER CODE END 0 */
/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */
  /* MCU Configuration--------------------------------------------------------*/
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  /* USER CODE BEGIN Init */
  /* USER CODE END Init */
  /* Configure the system clock */
  SystemClock_Config();
  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */
  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN;
  DWT_Init();
  ta_init(&TA_CFG);
  mode_touch();
  ADC1_Init_Reg();
  SPI2_Init_Reg();
  TIM2_Init_Reg();
  HAL_UART_Transmit(&huart2, (uint8_t *)"Smart Ink board1 start\r\n", 24, 100);
  uint32_t t_prn = 0, t_cpu = 0;
  char msg[80];
  /* USER CODE END 2 */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
    uint32_t now = HAL_GetTick();
    if (now - t_prn >= 100) {                    /* 0.1초마다 좌표 출력 */
      t_prn = now;
      int n = snprintf(msg, sizeof msg, "raw %4u %4u | xy %4u %4u touch %u speed %3u\r\n",
                       out.raw_x, out.raw_y, out.x, out.y, out.touch, out.speed);
      HAL_UART_Transmit(&huart2, (uint8_t *)msg, (uint16_t)n, 100);
    }
    if (now - t_cpu >= 1000) {                   /* 1초마다 CPU 점유율 */
      t_cpu = now;
      __disable_irq();
      uint32_t cyc = busy_cyc, tk = ticks;
      busy_cyc = 0; ticks = 0;
      __enable_irq();
      uint32_t permil = (uint32_t)((uint64_t)cyc * 1000U / SystemCoreClock);
      int n = snprintf(msg, sizeof msg, "[%s] CPU %lu.%lu%%  samples %lu/s\r\n",
                       USE_DMA ? "DMA" : "POLL", (unsigned long)(permil / 10U),
                       (unsigned long)(permil % 10U), (unsigned long)tk);
      HAL_UART_Transmit(&huart2, (uint8_t *)msg, (uint16_t)n, 100);
    }
  }
  /* USER CODE END 3 */
}
/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}
/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{
  /* USER CODE BEGIN USART2_Init 0 */
  /* USER CODE END USART2_Init 0 */
  /* USER CODE BEGIN USART2_Init 1 */
  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */
  /* USER CODE END USART2_Init 2 */
}
/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */
  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);
  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);
  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}
/* USER CODE BEGIN 4 */
/* USER CODE END 4 */
/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
