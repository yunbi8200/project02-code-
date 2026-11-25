/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * Smart Ink - STM32 #2 필압 보드 (Nucleo-F103RB)
  *
  * FSR -> ADC1 (TIM3 1kHz 트리거) -> DMA 순환 버퍼 [A 10개][B 10개]
  * -> 절반마다 평균 + press 판정 -> 10ms마다 압력 패킷 -> USART3 (DMA) -> STM32 #3
  *
  * 배선:  3.3V -- FSR -- PA0(A0) -- 10kΩ -- GND
  *        PB10 (D6, USART3 TX) -> STM32 #3 PB11,  GND 공통
  *        디버그 출력: USART2 (ST-LINK USB), 115200
  *
  * CubeMX: 기본 Nucleo-F103RB 설정(USART2)만 사용.
  *         ADC1, TIM3, DMA, USART3는 CubeMX에서 끄고 코드 재생성할 것
  *         (레지스터로 직접 설정, 인터럽트 함수도 여기 있음 -> 켜 두면 중복 정의 오류)
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "smartink_protocol.h"
/* USER CODE END Includes */
/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */
/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define USE_DMA        0        /* 1: ADC+DMA   0: Polling (CPU 점유율 비교용) */
#define SAMPLE_HZ      1000U    /* TIM3 트리거 = ADC 샘플링 주기 */
#define BLOCK          10U      /* 10개 평균 -> 100Hz로 #3에 전송 */
#define BUF_LEN        (2U * BLOCK)
/* 필압 범위는 #3 기준(smartink_stm3.c P_MIN 500 / P_MAX 2800, 웹 기본값도 같음)에 맞춘다.
 * #2는 원시값을 그대로 보내고, 아래 P3_ 값은 디버그 출력에서 "#3에서 어떻게 느껴지는지" 보여 주는 데만 쓴다.
 * 세게 눌러도 strong 이 안 나오면 풀다운 저항을 키우고(10k -> 22k), 살짝만 눌러도 100%면 줄인다(-> 4.7k). */
#define P3_MIN         300U
#define P3_MAX         1400U
#define P3_STRONG      (P3_MIN + (P3_MAX - P3_MIN) * 65U / 100U)   /* 1995: #3이 강한 소리로 바꾸는 값 */
/* press 판정 (히스테리시스) = 그려지느냐 마느냐. 실측해서 정할 것:
 * 펜을 살짝 대고 쓸 때 avg 보다 조금 아래. P3_MIN 보다 낮아도 된다 (그려지고 소리/굵기만 최소) */
#define PRESS_ON       350U
#define PRESS_OFF      300U
/* USER CODE END PD */
/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */
/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;
/* USER CODE BEGIN PV */
static volatile uint16_t adcBuf[BUF_LEN];       /* DMA 순환 버퍼 */
static volatile uint16_t pressAvg;              /* 최신 평균 */
static volatile uint16_t pressPeak;             /* 1초 동안 최댓값 (세게 누를 때 값 확인용) */
static volatile uint8_t  penPress;              /* 0/1 */
static volatile uint32_t busy_cyc;              /* ADC 처리에 쓴 CPU 사이클 */
static volatile uint32_t samples;               /* 처리한 샘플 수 */
static volatile uint32_t tx_count, tx_skip;     /* 송신 성공 / 이전 송신 중이라 건너뜀 */
static uint8_t txFrame[PKT_MAX_FRAME];
static uint8_t txSeq;
/* USER CODE END PV */
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */
/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* ───────── DWT 사이클 카운터 (CPU 점유율 측정) ───────── */
static void DWT_Init(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
}
/* ───────── USART3 TX (+ DMA1 채널2): #3으로 압력 패킷 ───────── */
static void USART3_Init_Reg(void)
{
  RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
  RCC->APB1ENR |= RCC_APB1ENR_USART3EN;
  RCC->AHBENR  |= RCC_AHBENR_DMA1EN;
  GPIOB->CRH = (GPIOB->CRH & ~(0xFU << 8)) | (0xBU << 8);          /* PB10 = AF 푸시풀 */
  USART3->BRR = (HAL_RCC_GetPCLK1Freq() + 57600U) / 115200U;       /* 115200 baud */
  USART3->CR3 = USART_CR3_DMAT;
  USART3->CR1 = USART_CR1_TE | USART_CR1_UE;
  DMA1_Channel2->CPAR = (uint32_t)&USART3->DR;
  DMA1_Channel2->CCR  = DMA_CCR_MINC | DMA_CCR_DIR;                 /* 메모리 -> USART, 8비트 */
}
static void send_pressure(void)
{
  if (DMA1_Channel2->CNDTR != 0U) { tx_skip++; return; }           /* 이전 패킷 송신 중 */
  pressure_data_t d = { txSeq++, pressAvg, penPress };
  uint16_t n = pkt_build_pressure(&d, txFrame);
  DMA1_Channel2->CCR  &= ~DMA_CCR_EN;
  DMA1_Channel2->CMAR  = (uint32_t)txFrame;
  DMA1_Channel2->CNDTR = n;
  DMA1_Channel2->CCR  |= DMA_CCR_EN;
  tx_count++;
}
/* 10개 평균 -> press 판정 -> 전송 */
static void process_block(const volatile uint16_t *b)
{
  uint32_t sum = 0;
  for (uint32_t i = 0; i < BLOCK; i++) sum += b[i];
  pressAvg = (uint16_t)(sum / BLOCK);
  if (pressAvg > pressPeak) pressPeak = pressAvg;
  if (!penPress && pressAvg >= PRESS_ON)       penPress = 1;
  else if (penPress && pressAvg <= PRESS_OFF)  penPress = 0;
  samples += BLOCK;
  send_pressure();
}
/* ───────── TIM3: 1kHz 업데이트 -> TRGO -> ADC 변환 시작 ───────── */
static void TIM3_Init_Reg(void)
{
  RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
  TIM3->CR1 = 0;
  TIM3->PSC = SystemCoreClock / 1000000U - 1U;                      /* 1MHz 카운트 */
  TIM3->ARR = 1000000U / SAMPLE_HZ - 1U;                            /* 1kHz */
  TIM3->CR2 = (TIM3->CR2 & ~TIM_CR2_MMS) | TIM_CR2_MMS_1;           /* TRGO = Update */
  TIM3->EGR = TIM_EGR_UG;
  TIM3->SR  = 0;
  TIM3->CR1 = TIM_CR1_CEN;
}
/* ───────── ADC1 채널0 (PA0), TIM3_TRGO 트리거 (+ DMA1 채널1 순환) ───────── */
static void ADC1_Init_Reg(void)
{
  RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_ADCPRE) | RCC_CFGR_ADCPRE_DIV6;   /* ADC 클럭 14MHz 이하 */
  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_ADC1EN;
  GPIOA->CRL &= ~0xFU;                                                /* PA0 = 아날로그 */
  ADC1->CR1   = 0;
  ADC1->SQR1  = 0;                                                    /* 변환 1개 */
  ADC1->SQR3  = 0;                                                    /* 채널 0 */
  ADC1->SMPR2 = (ADC1->SMPR2 & ~7U) | 6U;                             /* 71.5 사이클 */
  ADC1->CR2   = ADC_CR2_ADON;
  for (volatile int i = 0; i < 1000; i++) {}
  ADC1->CR2 |= ADC_CR2_RSTCAL; while (ADC1->CR2 & ADC_CR2_RSTCAL) {}
  ADC1->CR2 |= ADC_CR2_CAL;    while (ADC1->CR2 & ADC_CR2_CAL) {}
#if USE_DMA
  RCC->AHBENR |= RCC_AHBENR_DMA1EN;
  DMA1_Channel1->CCR   = 0;
  DMA1_Channel1->CPAR  = (uint32_t)&ADC1->DR;
  DMA1_Channel1->CMAR  = (uint32_t)adcBuf;
  DMA1_Channel1->CNDTR = BUF_LEN;
  DMA1->IFCR = DMA_IFCR_CGIF1;
  DMA1_Channel1->CCR   = DMA_CCR_CIRC | DMA_CCR_MINC | DMA_CCR_PSIZE_0 | DMA_CCR_MSIZE_0
                       | DMA_CCR_HTIE | DMA_CCR_TCIE;                 /* 절반/완료 인터럽트 */
  DMA1_Channel1->CCR  |= DMA_CCR_EN;
  NVIC_SetPriority(DMA1_Channel1_IRQn, 1);
  NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  ADC1->CR2 |= ADC_CR2_DMA;
#endif
  ADC1->CR2 |= (4U << ADC_CR2_EXTSEL_Pos) | ADC_CR2_EXTTRIG;          /* EXTSEL=100: TIM3_TRGO */
}
#if USE_DMA
/* 앞 절반이 차면 A 처리, 끝까지 차면 B 처리 (그동안 DMA는 다른 절반을 채움) */
void DMA1_Channel1_IRQHandler(void)
{
  uint32_t t0 = DWT->CYCCNT;
  uint32_t isr = DMA1->ISR;
  if (isr & DMA_ISR_HTIF1) { DMA1->IFCR = DMA_IFCR_CHTIF1; process_block(&adcBuf[0]); }
  if (isr & DMA_ISR_TCIF1) { DMA1->IFCR = DMA_IFCR_CTCIF1; process_block(&adcBuf[BLOCK]); }
  busy_cyc += DWT->CYCCNT - t0;
}
#else
/* Polling: 변환이 끝날 때까지 CPU가 기다렸다가 직접 읽음 */
static void poll_adc(void)
{
  static uint32_t idx;
  uint32_t t0 = DWT->CYCCNT;
  while (!(ADC1->SR & ADC_SR_EOC)) {}
  adcBuf[idx++] = (uint16_t)ADC1->DR;
  if (idx == BLOCK) { idx = 0; process_block(&adcBuf[0]); }
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
  DWT_Init();
  USART3_Init_Reg();
  ADC1_Init_Reg();
  TIM3_Init_Reg();
  HAL_UART_Transmit(&huart2, (uint8_t *)"Smart Ink board2 start\r\n", 24, 100);
  uint32_t t_prn = 0, t_cpu = 0;
  char msg[112];
  /* USER CODE END 2 */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
#if !USE_DMA
    poll_adc();
#endif
    uint32_t now = HAL_GetTick();
    if (now - t_prn >= 100) {                    /* 0.1초마다 필압 */
      t_prn = now;
      uint16_t a = pressAvg;
      uint32_t pct = (a <= P3_MIN) ? 0U : (a >= P3_MAX) ? 100U : (uint32_t)(a - P3_MIN) * 100U / (P3_MAX - P3_MIN);
      const char *zone = !penPress ? "none" : (a < P3_MIN) ? "draw(min)" : (a < P3_STRONG) ? "weak" : "strong";
      int n = snprintf(msg, sizeof msg, "avg %4u press %u | #3 %3lu%% %s\r\n",
                       a, penPress, (unsigned long)pct, zone);
      HAL_UART_Transmit(&huart2, (uint8_t *)msg, (uint16_t)n, 100);
    }
    if (now - t_cpu >= 1000) {                   /* 1초마다 CPU 점유율 */
      t_cpu = now;
      __disable_irq();
      uint32_t cyc = busy_cyc, sm = samples, tx = tx_count, sk = tx_skip, pk = pressPeak;
      busy_cyc = 0; samples = 0; tx_count = 0; tx_skip = 0; pressPeak = 0;
      __enable_irq();
      uint32_t permil = (uint32_t)((uint64_t)cyc * 1000U / SystemCoreClock);
      int n = snprintf(msg, sizeof msg, "[%s] CPU %lu.%lu%%  samples %lu/s  tx %lu/s  skip %lu  peak %lu\r\n",
                       USE_DMA ? "DMA" : "POLL", (unsigned long)(permil / 10U),
                       (unsigned long)(permil % 10U), (unsigned long)sm,
                       (unsigned long)tx, (unsigned long)sk, (unsigned long)pk);
      HAL_UART_Transmit(&huart2, (uint8_t *)msg, (uint16_t)n, 100);
    }
  }
  /* USER CODE END 3 */
}
/**
  * @brief System Clock Configuration (조원 프로젝트와 같은 HSE 72MHz)
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
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
}
/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}
/* USER CODE BEGIN 4 */
/* USER CODE END 4 */
/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}
#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */
