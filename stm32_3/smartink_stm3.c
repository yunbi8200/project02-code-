/*
 * smartink_stm3.c
 * STM32 #3 : 좌표(#1) + 압력(#2) + 모드(Arduino) 수신 -> 필기 상태 판단
 *            -> PC 로 통합 패킷 전송 + 필압에 맞는 필기음 생성 (PWM + DMA)
 *
 *   SPI2 슬레이브 (PB12 NSS, PB13 SCK, PB15 MOSI) <- STM32 #1 (마스터) 좌표 패킷
 *   USART1 RX (PA10) <- Arduino TX(D3, SoftwareSerial 19200) 모드 패킷 (5V 직결, PA10 은 FT 핀)
 *   USART3 RX (PB11) <- STM32 #2
 *   USART2 TX (PA2)  -> PC (Nucleo ST-LINK 가상 COM)
 *   TIM1_CH1 (PA8)   -> RC 저역통과 필터 -> LM386 앰프 -> 스피커
 *
 * HAL 함수 없이 레지스터만 사용한다 (클럭, GPIO, USART, SPI, TIM, DMA, NVIC 모두).
 * main.c 에서:
 *   void smartink_init(void);   void smartink_loop(void);   (선언)
 *   USER CODE 2 : smartink_init();      while(1) USER CODE 3 : smartink_loop();
 *
 * CubeMX 에서 USART1, USART3, SPI2, TIM1, TIM2, DMA 는 켜지 말 것 (IRQHandler 중복 정의).
 */
#include "stm32f1xx.h"
#include "smartink_protocol.h"
/* 필기음 데이터 크기. 실제 데이터는 파일 맨 아래.
 * 아래 두 구간은 tools/make_sound_data.py 가 자동으로 다시 쓴다 (직접 수정 X) */
/* @@SOUND_SIZE_BEGIN@@ */
#define SOUND_RATE      16000u
#define SND_STRONG_LEN  13600u
#define SND_WEAK_LEN    40000u
static const int8_t snd_strong[SND_STRONG_LEN];
static const int8_t snd_weak[SND_WEAK_LEN];
/* @@SOUND_SIZE_END@@ */
/* 1 : 가상 센서 + 가상 Arduino 패킷으로 테스트 (센서 도착 전)
 * 0 : 실제 #1, #2, Arduino 에서 수신 */
#ifndef SIM_MODE
#define SIM_MODE            0
#endif
#define UART_BAUD           115200u   /* #2(USART3), PC(USART2) */
#define ARDUINO_BAUD        19200u    /* USART1 <- Arduino SoftwareSerial(D3) 과 같은 값 */
#define OUTPUT_PERIOD_MS    10u       /* PC 패킷 100 Hz */
#define STALE_TIMEOUT_MS    50u       /* #1 좌표: 10 ms 마다 옴 -> 이 시간 동안 안 오면 끊김 */
#define PRESS_TIMEOUT_MS    250u      /* #2 필압: 100 ms 마다 옴 -> 2번 놓치면 끊김 */
#define MODE_TIMEOUT_MS     200u      /* Arduino 는 50 ms 마다 보냄 -> 4번 놓치면 끊김 */
#define RX_RING_SIZE        256u      /* 2의 거듭제곱 */
volatile uint32_t usart3_rx_count = 0;
/* ---- 필기음 조정값 (실제 펜/스피커로 들어보고 조정) ---- */
#define P_MIN               300       /* 이 FSR 값 이하 = 필압 0  (실측 범위 300~1300, 보통 800) */
#define P_MAX               1500      /* 이 FSR 값 이상 = 필압 1 */
#define P_STRONG_ON         18022     /* 0.55 (Q15) = 960: 보통(800)보다 세게 누르면 강한 소리 */
#define P_STRONG_OFF        14746     /* 0.45 (Q15) = 840: 보통 세기로 돌아오면 약한 소리 */
#define VOL_MIN             1638      /* 0.05 (Q15): 가장 약하게 쓸 때 음량. 음량 = 최소 + (1-최소) x 필압^2 */
#define VOL_MAX             32767     /* 1.0 */
#define XF_LEN              (SOUND_RATE * 150u / 1000u)   /* 크로스페이드 150 ms (미리듣기 값) */
#define WEAK_IN_STRONG_LEN  (SOUND_RATE / 2u)             /* 강 상태 안의 약 구간 0.5 s (길면 강한 소리가 잘 안 들림) */
#define WEAK_IN_STRONG_GAIN 256       /* Q8. 256 = 미리듣기의 음량비 그대로 */
#define PWM_CARRIER_HZ      64000u    /* 가청 대역 밖. 4 주기마다 샘플 1개 = 16 kHz */
#define AUDIO_HALF          256u      /* 16 kHz 에서 16 ms */
_Static_assert(PWM_CARRIER_HZ % SOUND_RATE == 0, "carrier must be a multiple of sample rate");
_Static_assert(WEAK_IN_STRONG_LEN <= SND_WEAK_LEN, "weak clip too short");
_Static_assert(2u * XF_LEN < SND_STRONG_LEN && 2u * XF_LEN < WEAK_IN_STRONG_LEN, "crossfade too long");
/* ================================================================== */
/* 클럭 / 1 ms 시간                                                    */
/* ================================================================== */
static uint32_t s_pclk1, s_pclk2, s_tim1clk, s_tim2clk;
static volatile uint32_t s_ms;
static void clock_init(void)
{
    /* CubeMX SystemClock_Config 를 이미 거쳤으면 그대로 사용 */
    if ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) {
        FLASH->ACR = FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY_1;   /* LATENCY=010: 48~72 MHz 는 2 wait */
        RCC->CFGR  = RCC_CFGR_PLLMULL16 | RCC_CFGR_PPRE1_DIV2;          /* HSI/2 x16 = 64 MHz, APB1 32 MHz */
        RCC->CR   |= RCC_CR_PLLON;
        while (!(RCC->CR & RCC_CR_PLLRDY)) {
        }
        RCC->CFGR |= RCC_CFGR_SW_PLL;
        while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) {
        }
    }
    SystemCoreClockUpdate();                         /* RCC 레지스터에서 SYSCLK 계산 (CMSIS) */
    uint32_t ppre1 = (RCC->CFGR & RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos;
    uint32_t ppre2 = (RCC->CFGR & RCC_CFGR_PPRE2) >> RCC_CFGR_PPRE2_Pos;
    s_pclk1   = SystemCoreClock >> APBPrescTable[ppre1];
    s_pclk2   = SystemCoreClock >> APBPrescTable[ppre2];
    s_tim2clk = (ppre1 & 4u) ? 2u * s_pclk1 : s_pclk1;  /* APB 분주가 있으면 타이머 클럭은 x2 */
    s_tim1clk = (ppre2 & 4u) ? 2u * s_pclk2 : s_pclk2;
}
/* TIM2 업데이트 인터럽트로 1 ms 카운터 (SysTick 은 CubeMX/HAL 이 쓸 수 있어서 피함) */
static void tick_init(void)
{
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    (void)RCC->APB1ENR;
    TIM2->PSC  = (uint16_t)(s_tim2clk / 1000000u - 1u);  /* 1 MHz */
    TIM2->ARR  = 1000u - 1u;                             /* 1 ms */
    TIM2->EGR  = TIM_EGR_UG;
    TIM2->SR   = 0;
    TIM2->DIER = TIM_DIER_UIE;
    NVIC_SetPriority(TIM2_IRQn, 3);
    NVIC_EnableIRQ(TIM2_IRQn);
    TIM2->CR1  = TIM_CR1_CEN;
}
void TIM2_IRQHandler(void)
{
    if (TIM2->SR & TIM_SR_UIF) {
        TIM2->SR = ~TIM_SR_UIF;    /* rc_w0: 0 을 쓴 비트만 지워짐. &= 는 그 사이 켜진 다른 플래그까지 지울 수 있음 */
        s_ms++;
    }
}
/* ================================================================== */
/* UART (레지스터 직접 설정)                                           */
/* ================================================================== */
typedef struct {
    volatile uint8_t  buf[RX_RING_SIZE];
    volatile uint16_t head;           /* ISR 이 씀 */
    volatile uint16_t tail;           /* main 이 씀 */
    volatile uint32_t errors;         /* ORE/FE/NE + 버퍼 넘침 */
} rx_ring_t;
static rx_ring_t s_rx_spi;            /* SPI2   <- #1 좌표 */
static rx_ring_t s_rx_mode;           /* USART1 <- Arduino 모드 */
static rx_ring_t s_rx_press;          /* USART3 <- #2 압력 */
/* ISR 에서 1바이트 넣기. 가득 차면 버리고 오류로 센다 (main 이 못 따라옴) */
static void ring_put(rx_ring_t *r, uint8_t b)
{
    uint16_t next = (uint16_t)((r->head + 1u) & (RX_RING_SIZE - 1u));
    if (next == r->tail) {
        r->errors++;
        return;
    }
    r->buf[r->head] = b;
    r->head = next;
}
static void usart_config(USART_TypeDef *u, uint32_t pclk, uint32_t baud, uint32_t cr1)
{
    u->CR1 = 0;
    u->CR2 = 0;                                  /* 1 stop bit */
    u->CR3 = 0;
    u->BRR = (pclk + baud / 2u) / baud;          /* 오버샘플링 16: BRR = pclk/baud */
    u->CR1 = cr1 | USART_CR1_UE;                 /* 8N1 */
}
static void uart_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_USART1EN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN | RCC_APB1ENR_USART3EN;
    (void)RCC->APB1ENR;
    /* F1 GPIO 는 핀당 CNF[1:0] MODE[1:0]. 필드를 먼저 지우고 값을 쓴다 */
    /* PA10 USART1_RX: floating input (CNF=01, MODE=00). Arduino 5V TX 직결이라
     * 내부 풀업을 끈다 (FT 핀이 VDD 이상을 견디려면 풀업/풀다운이 꺼져 있어야 함) */
    GPIOA->CRH = (GPIOA->CRH & ~(GPIO_CRH_CNF10 | GPIO_CRH_MODE10)) | GPIO_CRH_CNF10_0;
    /* PB11 USART3_RX: input pull-up (CNF=10, MODE=00, ODR=1) */
    GPIOB->CRH = (GPIOB->CRH & ~(GPIO_CRH_CNF11 | GPIO_CRH_MODE11)) | GPIO_CRH_CNF11_1;
    GPIOB->ODR |= GPIO_ODR_ODR11;
    /* PA2 USART2_TX: AF push-pull 50 MHz (CNF=10, MODE=11) */
    GPIOA->CRL = (GPIOA->CRL & ~(GPIO_CRL_CNF2 | GPIO_CRL_MODE2)) | GPIO_CRL_CNF2_1 | GPIO_CRL_MODE2;
    /* USART1 은 APB2, USART2/3 은 APB1 */
    usart_config(USART1, s_pclk2, ARDUINO_BAUD, USART_CR1_RE | USART_CR1_RXNEIE);
    usart_config(USART3, s_pclk1, UART_BAUD,    USART_CR1_RE | USART_CR1_RXNEIE);
    usart_config(USART2, s_pclk1, UART_BAUD,    USART_CR1_TE);
    NVIC_SetPriority(USART1_IRQn, 1);
    NVIC_SetPriority(USART3_IRQn, 1);
    NVIC_EnableIRQ(USART1_IRQn);
    NVIC_EnableIRQ(USART3_IRQn);
}
static void usart_rx_isr(USART_TypeDef *u, rx_ring_t *r)
{
    uint32_t sr = u->SR;
    if (!(sr & (USART_SR_RXNE | USART_SR_ORE))) {   /* RXNEIE 는 ORE 에도 인터럽트를 건다 */
        return;
    }
    uint8_t b = (uint8_t)u->DR;                     /* SR -> DR 읽기로 ORE/FE/NE 해제 */
    if (sr & (USART_SR_ORE | USART_SR_FE | USART_SR_NE)) {
        r->errors++;
        if (sr & (USART_SR_FE | USART_SR_NE)) {
            return;                                 /* 깨진 바이트는 버림 (ORE 는 DR 값이 정상) */
        }
    }
    ring_put(r, b);
}
void USART1_IRQHandler(void) { usart_rx_isr(USART1, &s_rx_mode); }
void USART3_IRQHandler(void)
{
    usart3_rx_count++;
    usart_rx_isr(USART3, &s_rx_press);
}
static int rx_ring_get(rx_ring_t *r, uint8_t *out)
{
    if (r->tail == r->head) {
        return 0;
    }
    *out = r->buf[r->tail];
    r->tail = (uint16_t)((r->tail + 1u) & (RX_RING_SIZE - 1u));
    return 1;
}
static void uart2_write(const uint8_t *data, uint16_t len)
{
    while (len--) {
        while (!(USART2->SR & USART_SR_TXE)) {
        }
        USART2->DR = *data++;
    }
}
/* ================================================================== */
/* SPI2 슬레이브 수신 (#1 마스터 -> #3, 좌표 패킷)                      */
/*   PB12 NSS(입력, 풀업)  PB13 SCK(입력)  PB15 MOSI(입력). MISO 미사용  */
/*   모드 0 (CPOL=0, CPHA=0), 8비트, MSB 먼저, 수신 전용(RXONLY).        */
/*   받은 바이트는 UART 와 같은 링버퍼에 넣고, 패킷 구분은 AA 55 파서가  */
/*   한다. 데이터가 적고 DMA1 채널5 는 오디오가 쓰므로 DMA 는 안 쓴다.   */
/* ================================================================== */
static void spi_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
    RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
    (void)RCC->APB1ENR;
    /* PB13 SCK, PB15 MOSI: floating input (CNF=01, MODE=00) */
    GPIOB->CRH = (GPIOB->CRH & ~(GPIO_CRH_CNF13 | GPIO_CRH_MODE13 | GPIO_CRH_CNF15 | GPIO_CRH_MODE15))
               | GPIO_CRH_CNF13_0 | GPIO_CRH_CNF15_0;
    /* PB12 NSS: input pull-up (CNF=10, MODE=00, ODR=1). #1 이 빠져 있으면 '선택 안 됨' */
    GPIOB->CRH = (GPIOB->CRH & ~(GPIO_CRH_CNF12 | GPIO_CRH_MODE12)) | GPIO_CRH_CNF12_1;
    GPIOB->ODR |= GPIO_ODR_ODR12;
    SPI2->CR1 = 0;                                    /* 슬레이브, 모드 0, 8비트, MSB, 하드웨어 NSS */
    SPI2->CR1 = SPI_CR1_RXONLY;                       /* MISO 를 구동하지 않음 */
    SPI2->CR2 = SPI_CR2_RXNEIE | SPI_CR2_ERRIE;
    SPI2->CR1 |= SPI_CR1_SPE;
    NVIC_SetPriority(SPI2_IRQn, 1);
    NVIC_EnableIRQ(SPI2_IRQn);
}
void SPI2_IRQHandler(void)
{
    uint32_t sr = SPI2->SR;
    if (sr & SPI_SR_RXNE) {
        ring_put(&s_rx_spi, (uint8_t)SPI2->DR);
    }
    if (sr & SPI_SR_OVR) {                            /* DR -> SR 읽기로 OVR 해제 */
        (void)SPI2->DR;
        (void)SPI2->SR;
        s_rx_spi.errors++;
    }
}
/* ================================================================== */
/* 필기음 출력: TIM1 PWM + DMA 순환 버퍼                                */
/*   DMA 가 [A][B] 중 한쪽을 PWM 비교 레지스터로 보내는 동안            */
/*   CPU 는 다른 쪽을 채운다. 반완료(HT) -> A 채움, 완료(TC) -> B 채움   */
/* ================================================================== */
static uint16_t          s_audio_buf[2u * AUDIO_HALF];
static volatile int8_t   s_fill_req = -1;     /* 채울 절반 0/1, 없으면 -1 */
static volatile uint32_t s_audio_miss;        /* 마감 실패: 이전 요청을 처리하기 전에 다음 요청이 옴 */
static int32_t           s_pwm_mid, s_pwm_top;
static void audio_init(void)
{
    RCC->AHBENR  |= RCC_AHBENR_DMA1EN;
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN | RCC_APB2ENR_IOPAEN;
    (void)RCC->APB2ENR;
    /* PA8 TIM1_CH1: AF push-pull 50 MHz (CNF=10, MODE=11). 기본 매핑이라 AFIO 리맵 불필요 */
    GPIOA->CRH = (GPIOA->CRH & ~(GPIO_CRH_CNF8 | GPIO_CRH_MODE8)) | GPIO_CRH_CNF8_1 | GPIO_CRH_MODE8;
    s_pwm_top = (int32_t)(s_tim1clk / PWM_CARRIER_HZ) - 1;   /* 64 MHz -> 999 */
    s_pwm_mid = (s_pwm_top + 1) / 2;
    for (uint32_t i = 0; i < 2u * AUDIO_HALF; i++) {
        s_audio_buf[i] = (uint16_t)s_pwm_mid;        /* 무음 = 중앙 듀티 (0 이 아님) */
    }
    TIM1->CR1   = TIM_CR1_ARPE;
    TIM1->PSC   = 0;
    TIM1->ARR   = (uint16_t)s_pwm_top;
    TIM1->RCR   = PWM_CARRIER_HZ / SOUND_RATE - 1u;  /* update(=DMA 요청) 를 4 주기마다 */
    TIM1->CCR1  = (uint16_t)s_pwm_mid;
    TIM1->CCMR1 = TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1PE;  /* PWM mode 1 */
    TIM1->CCER  = TIM_CCER_CC1E;
    TIM1->BDTR  = TIM_BDTR_MOE;                      /* TIM1(고급 타이머)은 MOE 가 있어야 출력됨 */
    TIM1->EGR   = TIM_EGR_UG;                        /* PSC/RCR 로드. UDE 켜기 전에 해야 DMA 안 튐 */
    TIM1->SR    = 0;
    DMA1_Channel5->CCR   = 0;                        /* F103: TIM1_UP = DMA1 채널 5 */
    DMA1_Channel5->CPAR  = (uint32_t)&TIM1->CCR1;
    DMA1_Channel5->CMAR  = (uint32_t)s_audio_buf;
    DMA1_Channel5->CNDTR = 2u * AUDIO_HALF;
    DMA1_Channel5->CCR   = DMA_CCR_DIR | DMA_CCR_CIRC | DMA_CCR_MINC |
                           DMA_CCR_PSIZE_0 | DMA_CCR_MSIZE_0 |      /* 16 bit -> 16 bit */
                           DMA_CCR_HTIE | DMA_CCR_TCIE | DMA_CCR_PL_1;
    DMA1_Channel5->CCR  |= DMA_CCR_EN;
    NVIC_SetPriority(DMA1_Channel5_IRQn, 2);
    NVIC_EnableIRQ(DMA1_Channel5_IRQn);
    TIM1->DIER = TIM_DIER_UDE;
    TIM1->CR1 |= TIM_CR1_CEN;
}
void DMA1_Channel5_IRQHandler(void)
{
    uint32_t isr = DMA1->ISR;
    int8_t half = -1;
    if (isr & DMA_ISR_HTIF5) {                       /* 앞 절반 출력 끝 -> 앞 절반 채우기 */
        DMA1->IFCR = DMA_IFCR_CHTIF5;
        half = 0;
    }
    if (isr & DMA_ISR_TCIF5) {                       /* 뒤 절반 출력 끝 -> 뒤 절반 채우기 */
        DMA1->IFCR = DMA_IFCR_CTCIF5;
        half = 1;
    }
    if (half >= 0) {
        if (s_fill_req >= 0) {
            s_audio_miss++;
        }
        s_fill_req = half;
    }
}
/* ---- 음원 재생 ----
 * 필압 상태(WEAK/STRONG)와 재생 구간(voice)은 별개다.
 * STRONG 상태 안에서는 강 -> (키운) 약 -> 강 ... 으로 구간만 바뀐다. */
enum { V_WEAK, V_WEAK_IN_STRONG, V_STRONG };
static const int8_t *const v_data[] = { snd_weak, snd_weak, snd_strong };
static const uint32_t      v_len[]  = { SND_WEAK_LEN, WEAK_IN_STRONG_LEN, SND_STRONG_LEN };
static const int32_t       v_gain[] = { 256, WEAK_IN_STRONG_GAIN, 256 };          /* Q8 */
static const uint8_t       v_next[] = { V_WEAK, V_STRONG, V_WEAK_IN_STRONG };     /* 끝나면 이어서 */
typedef struct {
    uint8_t  kind;
    uint32_t pos;
} voice_t;
static voice_t s_cur, s_prev;      /* s_prev = 크로스페이드로 사라지는 쪽 */
static uint32_t s_xf;              /* 0: 전환 없음, 1..XF_LEN: 전환 진행 */
static uint8_t  s_strong;          /* 필압 상태 */
static int32_t  s_ctrl_p;          /* 소리 제어용 필압 (Q15) */
static int32_t  s_vol;             /* 현재 음량 (Q15) */
static void start_xf(uint8_t kind)
{
    s_prev     = s_cur;
    s_cur.kind = kind;
    s_cur.pos  = 0;
    s_xf       = 1;
}
static int32_t voice_next(voice_t *v)
{
    int32_t s = v_data[v->kind][v->pos] * v_gain[v->kind] / 256;
    if (++v->pos >= v_len[v->kind]) {
        v->pos = 0;                /* 사라지는 쪽이 끝에 닿으면 처음부터 (전환 중에만 발생) */
    }
    return s;
}
/* 출력 버퍼 절반(16 ms)을 채운다. active = 유효한 필기 + 필기 모드 */
static void audio_fill(uint16_t *dst, uint8_t active, uint16_t pressure)
{
    /* 1) 필압 0~1 (Q15) + 가벼운 평활 (2번이 이미 필터링함) */
    int32_t p = ((int32_t)pressure - P_MIN) * 32768 / (P_MAX - P_MIN);
    p = (p < 0) ? 0 : (p > 32767) ? 32767 : p;
    s_ctrl_p = (s_vol == 0) ? p : s_ctrl_p + (p - s_ctrl_p) / 2;   /* 새 획은 바로 시작 */
    /* 2) 약/강: 진입 0.65, 해제 0.55 (경계에서 떨림 방지) */
    if (active) {
        if (!s_strong && s_ctrl_p >= P_STRONG_ON) {
            s_strong = 1;
        } else if (s_strong && s_ctrl_p <= P_STRONG_OFF) {
            s_strong = 0;
        }
    }
    /* 3) 상태와 재생 구간이 다르면 전환. 전환 중이면 끝난 뒤 다음 블록에서 */
    if (s_xf == 0 && s_strong != (s_cur.kind != V_WEAK)) {
        start_xf(s_strong ? V_STRONG : V_WEAK);
    }
    /* 4) 목표 음량 = 최소 + (최대-최소) x 필압^2. 제곱이라 세게 쓸수록 크게 달라져 귀로 구분된다
     *    (P_MAX 1500 기준 400: 6%, 800: 22%, 960: 34%, 1500: 100%). 펜을 떼면 0 으로 짧게 감쇠 (~100 ms) */
    int32_t p2      = (s_ctrl_p * s_ctrl_p) >> 15;
    int32_t target  = active ? VOL_MIN + (VOL_MAX - VOL_MIN) * (p2 >> 3) / 4096 : 0;
    int32_t vol_end = s_vol + (target - s_vol) / 2;
    if (vol_end - target < 512 && target - vol_end < 512) {
        vol_end = target;
    }
    int32_t vol = s_vol;
    int32_t dv  = (vol_end - s_vol) / (int32_t)AUDIO_HALF;          /* 블록 안에서 선형 변화 -> 클릭 방지 */
    /* 5) 샘플 생성 */
    for (uint32_t i = 0; i < AUDIO_HALF; i++) {
        int32_t s = voice_next(&s_cur);
        if (s_xf) {                                                 /* 출력 = (1-r) x 이전 + r x 다음 */
            int32_t old = voice_next(&s_prev);
            s = (s * (int32_t)s_xf + old * (int32_t)(XF_LEN - s_xf)) / (int32_t)XF_LEN;
            if (++s_xf > XF_LEN) {
                s_xf = 0;
            }
        } else if (s_cur.pos >= v_len[s_cur.kind] - XF_LEN) {       /* 끝과 다음 시작을 겹쳐서 연결 */
            start_xf(v_next[s_cur.kind]);
        }
        vol += dv;
        int32_t out = s_pwm_mid + ((((s * vol) >> 7) * s_pwm_mid) >> 15);
        dst[i] = (uint16_t)((out < 0) ? 0 : (out > s_pwm_top) ? s_pwm_top : out);
    }
    s_vol = vol_end;
    /* 6) 완전히 꺼졌으면 다음 획은 약한 소리 처음부터 */
    if (!active && s_vol == 0) {
        s_cur.kind = V_WEAK;
        s_cur.pos  = 0;
        s_xf       = 0;
        s_strong   = 0;
    }
}
/* ================================================================== */
/* 데이터 통합                                                         */
/* ================================================================== */
static pkt_parser_t    s_parser_spi;                /* #1 (SPI2) */
static pkt_parser_t    s_parser_mode;               /* Arduino (USART1) */
static pkt_parser_t    s_parser_press;              /* #2 (USART3) */
static touch_data_t    s_touch;
static pressure_data_t s_press;
static mode_data_t     s_mode;                      /* 0 으로 시작 = 펜, 음소거 없음 */
static uint32_t        s_touch_ms, s_press_ms, s_mode_ms;
static uint8_t         s_touch_valid, s_press_valid, s_mode_valid;
static uint32_t        s_mode_ok;                   /* 디버거 확인용 */
static uint8_t         s_seq;
static uint32_t        s_last_out_ms;
static uint32_t        s_err_snapshot;
/* 라인이 아니라 TYPE 으로 구분한다 (어느 통로로 와도 TYPE 대로 처리) */
static void feed(pkt_parser_t *p, uint8_t b, uint32_t now)
{
    if (!pkt_parser_feed(p, b)) {
        return;
    }
    if (p->type == PKT_TYPE_TOUCH && pkt_decode_touch(p->payload, p->len, &s_touch) == 0) {
        s_touch_ms = now;
        s_touch_valid = 1;
    } else if (p->type == PKT_TYPE_PRESSURE && pkt_decode_pressure(p->payload, p->len, &s_press) == 0) {
        s_press_ms = now;
        s_press_valid = 1;
    } else if (p->type == PKT_TYPE_MODE && pkt_decode_mode(p->payload, p->len, &s_mode) == 0) {
        s_mode_ms = now;                            /* Arduino 모드 */
        s_mode_valid = 1;
        s_mode_ok++;
    }
}
/* 최신 수신값 -> 하나의 필기 상태. 끊긴 데이터는 0 으로 취급해 소리가 계속 나지 않게 한다 */
static void build_state(uint32_t now, sensor_data_t *d)
{
    uint8_t touch_fresh = s_touch_valid && (uint32_t)(now - s_touch_ms) <= STALE_TIMEOUT_MS;
    uint8_t press_fresh = s_press_valid && (uint32_t)(now - s_press_ms) <= PRESS_TIMEOUT_MS;
    uint8_t mode_fresh  = s_mode_valid  && (uint32_t)(now - s_mode_ms)  <= MODE_TIMEOUT_MS;
    d->x        = s_touch.x;                        /* 끊겨도 마지막 좌표 유지 */
    d->y        = s_touch.y;
    d->touch    = touch_fresh ? s_touch.touch : 0u;
    d->pressure = press_fresh ? s_press.pressure : 0u;
    d->press    = press_fresh ? s_press.press : 0u;
    d->flags    = 0;
    /* 터치 + 압력 둘 다일 때만 유효. 모드를 모르면(끊김) 그리기·지우기 잠시 중단 */
    if (d->touch && d->press && mode_fresh)   d->flags |= SENSOR_FLAG_DRAW;
    if (!touch_fresh)                         d->flags |= SENSOR_FLAG_TOUCH_STALE;
    if (!press_fresh)                         d->flags |= SENSOR_FLAG_PRESS_STALE;
    if (!mode_fresh)                          d->flags |= SENSOR_FLAG_MODE_STALE;
    if (s_mode.mode == MODE_ERASER)           d->flags |= SENSOR_FLAG_ERASER;     /* 마지막 정상 모드 */
    if (s_mode.options & MODE_OPT_MUTE)       d->flags |= SENSOR_FLAG_MUTE;
}
static void send_sensor_packet(sensor_data_t *d)
{
    d->seq = s_seq++;
    uint32_t err = s_parser_spi.errors + s_parser_mode.errors + s_parser_press.errors +
                   s_rx_spi.errors + s_rx_mode.errors + s_rx_press.errors;
    if (err != s_err_snapshot) {
        d->flags |= SENSOR_FLAG_RX_ERROR;
        s_err_snapshot = err;
    }
    uint8_t frame[PKT_MAX_FRAME];
    uart2_write(frame, pkt_build_sensor(d, frame));
}
/* ================================================================== */
/* 가상 센서 (SIM_MODE 1)                                              */
/*  6초 주기: 0.3s 펜 대기 -> 1.5s 동안 필압 증가(약 -> 강)              */
/*            -> 2.5s 강하게 유지(강 -> 약 -> 강 확인) -> 1s 감소 -> 뗌   */
/*  좌표 37번째마다 비트 반전, 압력 53번째마다 잘린 프레임,            */
/*  3초마다 쓰레기 바이트 -> 오류 검출과 재동기화 확인                  */
/*  디버거 Live Expressions 로 s_parser_spi / s_parser_press 의        */
/*  ok_count, errors 를 보면 된다. (초당 대략 touch ok 195/err 5,       */
/*  press ok 96/err 2 — 잘린 프레임은 다음 프레임 하나를 같이 잃는다)  */
/*  Arduino 모드: 50 ms 마다, 3주기 중 마지막 1주기(12~18 s)는 지우개,  */
/*  23번째마다 CRC 오류 -> s_mode_ok 초당 약 19, s_parser_mode.errors 약 1 */
/*  (실제 Arduino 를 같이 연결하면 두 모드가 섞이니 SIM 0 으로)          */
/* ================================================================== */
#if SIM_MODE
static uint16_t tri_wave(uint32_t t, uint32_t period, uint16_t amp)
{
    uint32_t ph = t % period, half = period / 2u;
    return (uint16_t)(((ph < half) ? ph : (period - ph)) * amp / half);
}
static void sim_send(pkt_parser_t *p, const uint8_t *f, uint16_t n, uint32_t now)
{
    while (n--) {
        feed(p, *f++, now);
    }
}
static void sim_poll(uint32_t now)
{
    static uint32_t touch_ms, press_ms, junk_ms, mode_ms;
    static uint16_t touch_cnt, press_cnt;
    static uint8_t  mode_seq;
    uint8_t frame[PKT_MAX_FRAME];
    uint16_t n;
    uint32_t cyc = now % 6000u;
    if ((uint32_t)(now - touch_ms) >= 5u) {
        touch_ms = now;
        touch_data_t t = {
            .x = (uint16_t)(500u + tri_wave(now, 3000u, 3000u)),
            .y = (uint16_t)(500u + tri_wave(now, 1900u, 3000u)),
            .touch = cyc < 5500u,
        };
        n = pkt_build_touch(&t, frame);
        if (++touch_cnt % 37u == 0u) {
            frame[5] ^= 0x10u;                      /* CRC 오류 */
        }
        sim_send(&s_parser_spi, frame, n, now);
    }
    if ((uint32_t)(now - press_ms) >= 10u) {
        press_ms = now;
        uint32_t t = cyc - 300u;                    /* cyc < 300 이면 매우 큰 값 -> 뗀 상태 */
        pressure_data_t p;
        p.pressure = (t < 1500u) ? (uint16_t)(300u + t * 1000u / 1500u)    /* 실측 범위 300~1300 */
                   : (t < 4000u) ? 1300u
                   : (t < 5000u) ? (uint16_t)(1300u - (t - 4000u) * 1000u / 1000u)
                   : 50u;                           /* 뗐을 때 잡음 수준 */
        p.press = p.pressure >= 400u;               /* #2 의 press ON 기준과 같게 */
        n = pkt_build_pressure(&p, frame);
        if (++press_cnt % 53u == 0u) {
            n -= 2u;                                /* 잘린 프레임 */
        }
        sim_send(&s_parser_press, frame, n, now);
    }
    if ((uint32_t)(now - junk_ms) >= 3000u) {
        junk_ms = now;
        static const uint8_t junk[] = { 0x00, 0xAA, 0x13, 0xFF, 0x55, 0xAA };
        sim_send(&s_parser_spi, junk, sizeof(junk), now);
    }
    if ((uint32_t)(now - mode_ms) >= 50u) {
        mode_ms = now;
        mode_data_t m = {
            .seq     = mode_seq++,
            .mode    = ((now / 6000u) % 3u == 2u) ? MODE_ERASER : MODE_PEN,
            .options = 0,
        };
        n = pkt_build_mode(&m, frame);
        if (m.seq % 23u == 0u) {
            frame[7] ^= 0x01u;                      /* CRC 오류 */
        }
        sim_send(&s_parser_mode, frame, n, now);
    }
}
#endif
/* SCK 펄스를 하나라도 놓치거나 잡음으로 하나 더 들어오면 비트가 밀려, 그 뒤로는
 * AA 55 자체를 못 찾는다 (오류 카운트도 거의 안 늘어 오류 기준 복구는 늦다).
 * 그래서 패킷이 끝날 때마다(바이트가 들어왔고, 다 처리했고, NSS 가 HIGH)
 *  1) SPI 를 껐다 켜서 다음 패킷을 항상 첫 비트부터 받고
 *  2) 파서에 남은 미완성 패킷(잘린 패킷)을 버린다 -> 다음 정상 패킷을 잃지 않는다. */
static void spi_resync(void)
{
    static uint16_t last_head;
    uint16_t head = s_rx_spi.head;
    if (head == last_head || !(GPIOB->IDR & GPIO_IDR_IDR12) || s_rx_spi.tail != head) {
        return;                                     /* 새 바이트 없음 / 전송 중 / 아직 덜 읽음 */
    }
    last_head = head;
    SPI2->CR1 &= ~SPI_CR1_SPE;
    SPI2->CR1 |= SPI_CR1_SPE;
    if (s_parser_spi.state != 0) {                  /* 패킷이 중간에 끝남 = 잘린 패킷 */
        s_parser_spi.errors++;
        s_parser_spi.state = 0;
    }
}
/* ================================================================== */
void smartink_init(void)
{
    clock_init();
    tick_init();
    uart_init();
    spi_init();
    audio_init();
    s_last_out_ms = s_ms;
}
void smartink_loop(void)
{
    uint32_t now = s_ms;
    uint8_t b;
    sensor_data_t d;
    while (rx_ring_get(&s_rx_spi,   &b)) feed(&s_parser_spi,   b, now);
    while (rx_ring_get(&s_rx_mode,  &b)) feed(&s_parser_mode,  b, now);
    while (rx_ring_get(&s_rx_press, &b)) feed(&s_parser_press, b, now);
    spi_resync();
#if SIM_MODE
    sim_poll(now);
#endif
    /* 오디오가 마감이 있으므로 먼저 */
    __disable_irq();
    int8_t half = s_fill_req;
    s_fill_req = -1;
    __enable_irq();
    if (half >= 0) {
        build_state(now, &d);
        uint8_t active = (d.flags & SENSOR_FLAG_DRAW) &&
                         !(d.flags & (SENSOR_FLAG_ERASER | SENSOR_FLAG_MUTE));   /* 펜 모드 + 음소거 아님 */
        audio_fill(&s_audio_buf[(uint32_t)half * AUDIO_HALF], active, d.pressure);
    }
    if ((uint32_t)(now - s_last_out_ms) >= OUTPUT_PERIOD_MS) {
        s_last_out_ms += OUTPUT_PERIOD_MS;
        build_state(now, &d);
        send_sensor_packet(&d);                     /* 15 B 블로킹 ≈ 1.3 ms, 오디오 마감 16 ms 안 */
    }
}
/* ================================================================== */
/* 필기음 데이터 (자동 생성, 원본: Trtle.T, CC0)                       */
/* ================================================================== */
/* @@SOUND_DATA_BEGIN@@ */
static const int8_t snd_strong[SND_STRONG_LEN] = {
    10, 17, 15, 13, 15, 21, 17, 12, 10, 6, -3, 6, -4, 7, 2, 2, 0, 8, 2, -8, -4, -9, -4, 3,
    9, 2, 6, -1, 0, -3, -2, -6, -14, -11, -4, -15, -15, -20, -19, -21, -18, -16, -23, -21, -21, -10, -5, -2,
    1, -5, -3, -6, -7, -12, -19, -20, -21, -8, -2, 3, 23, 4, 8, 6, -4, -4, -13, -8, -7, -7, 9, 6,
    15, 19, 18, 20, 10, 16, 2, 5, -4, -11, -3, -11, -8, 1, -6, 8, -7, 9, -15, -3, -10, -21, -7, -17,
    -12, -11, -12, 0, -11, 0, -12, -11, -8, -16, -5, -18, -14, -16, -15, -12, -18, -8, -9, 1, -4, -5, -4, -10,
    2, -6, -9, -13, -15, -6, -12, -9, -10, -15, -13, -17, -22, -26, -26, -16, -19, -12, -7, -17, -20, -21, -22, -23,
    -14, -20, -22, -20, -14, -20, -14, -11, -11, -8, -24, -15, -8, -10, -20, -22, -14, -22, -12, -6, 2, -5, 8, 13,
    10, 0, -1, 2, -6, 2, 4, -4, 1, 1, 1, 6, -3, -2, -14, -12, -9, -14, -8, -15, -13, -6, -6, -13,
    -9, -5, -5, 1, 12, -3, 4, 7, 5, 3, 4, 5, -9, 10, -1, -1, -2, -2, -6, -7, 1, 3, 0, 9,
    -3, 7, 10, 10, 9, 9, 14, 18, 29, 23, 21, 25, 22, 11, 14, 12, 16, 18, 20, 21, 30, 28, 22, 23,
    10, 13, 15, 6, 6, 3, 12, 13, 7, 17, 12, 17, 9, 14, 12, 12, 12, 11, 7, 8, 15, 14, 2, 8,
    -4, 12, 7, 6, 13, 15, 24, 22, 12, 4, 15, 19, 17, 13, 11, 21, 13, 31, 20, 20, 17, 22, 19, 20,
    16, 13, 18, 24, 32, 31, 25, 18, 19, 22, 12, -3, -1, -5, 0, 3, 16, 11, 20, 37, 27, 33, 32, 20,
    25, 14, 12, 8, 10, 9, -2, 12, 2, 8, -2, -8, 0, -16, -16, -21, -15, -24, -12, -14, -16, -20, -22, -15,
    -8, -3, -9, -2, -13, -14, 1, 0, -3, -13, 4, -4, 0, -15, -7, -20, -25, -22, -21, -32, -26, -24, -7, -15,
    -17, -30, -34, -16, 3, 14, 24, 10, -3, -5, -10, -10, -27, -31, -34, -14, -8, 11, 0, 4, -7, -4, -5, -23,
    -26, -29, -33, -12, -8, -6, 8, 13, 21, 24, 26, 15, 9, 5, 1, -9, -15, -13, -32, -20, -26, -21, -27, -15,
    -23, -13, -13, -20, -20, -13, -17, -3, 2, 22, 0, 17, 15, 17, 1, 9, 12, 4, 17, 15, 2, -12, -5, -8,
    -22, -5, -5, -13, 4, -8, -11, -15, 5, 10, 2, 15, -17, -13, -13, -22, -33, -22, -17, -24, -15, -19, -14, -20,
    -26, -21, -20, -28, -19, -32, -29, -6, 8, -3, -8, -9, -1, 12, 14, 10, 11, 5, 9, 7, 21, 44, 36, 61,
    33, 49, 45, 36, 22, 20, 28, 26, 9, 4, 7, 11, 20, 15, 0, 16, 16, 10, 10, 6, 7, -1, -4, -17,
    -10, 5, 1, -10, -12, -4, -9, -12, -24, -15, -17, -20, -28, -40, -34, -49, -50, -45, -47, -20, -16, -8, -10, -2,
    9, 7, 26, 16, 21, 12, 20, 11, 12, 15, 9, 17, 9, -4, -7, -8, 4, -4, 5, 1, -4, 5, 3, 3,
    4, -12, -20, -7, -14, -12, -12, -13, -3, -8, -3, -14, -12, -11, -13, 0, -2, 1, 1, -1, 3, 5, -1, 5,
    13, 17, 14, 16, 10, 6, 25, 24, 29, 27, 15, 3, 18, 14, 12, 19, 14, 9, 22, 3, 12, 15, 19, 25,
    8, 25, 30, 29, 34, 32, 33, -5, -67, -52, -36, -11, 11, 16, 31, 51, 69, 70, 51, 56, 42, 34, 22, 16,
    14, 30, 42, 27, 27, 15, 15, 15, 11, 15, 8, 15, 11, 2, 19, 8, -3, 12, 1, 5, 8, 2, 0, 3,
    3, 10, -1, -2, 0, -6, -13, -26, -4, -18, -21, -19, -10, -8, -6, -1, 0, 0, 1, -5, -2, 1, -3, 10,
    5, 4, -10, -3, 0, 1, -8, -10, -21, -31, -17, -22, -8, 2, -1, 9, 9, 7, 1, -6, 1, -5, -3, -9,
    -13, -11, -12, 3, -8, 1, 2, -5, -11, -17, -8, -27, -34, -39, -38, -23, -23, -20, -16, -12, -6, -13, -5, -13,
    -15, -19, -23, -17, -35, -32, -53, -64, -78, -76, -98, -87, -59, -40, -39, -32, -13, -14, 15, 15, 12, 24, 29, 18,
    15, 22, 29, 11, 23, 9, 5, 9, -10, -20, -20, -6, -18, -2, 4, -1, -2, -1, 2, -13, 7, 3, -7, -1,
    -5, 3, -6, -8, -10, -2, 7, -13, -19, -16, -6, -9, -2, -17, -40, -28, -11, -8, 16, -2, 8, 17, 14, 32,
    35, 31, 17, 16, 21, 9, 6, 6, 6, -4, -12, 1, -21, -22, -16, -7, -1, -4, -1, -8, 6, 2, 7, -1,
    -9, -25, -7, -9, -5, 2, -3, -1, 14, 7, 4, -1, 18, 2, 2, 10, 3, 32, 33, 38, 34, 30, 42, 37,
    39, 22, 17, 19, 29, 23, 18, 23, 15, 17, 17, -6, -5, 4, 4, -7, 3, 16, 18, 6, 21, 23, 21, 29,
    6, 13, 24, 15, 7, 8, 2, 10, 12, 22, 12, 20, 18, -3, 2, -4, 1, -5, -1, -2, 1, 8, -2, -14,
    -4, -8, -15, -9, -1, 2, 2, -2, -2, -1, -1, -10, -10, -12, -7, -12, -10, -19, -25, -19, -14, -32, -25, -31,
    -46, -45, -49, -46, -43, -28, -29, -34, -13, -27, -17, -20, -23, -28, -36, -27, -21, -13, -14, -22, -4, 1, 10, -11,
    -20, -16, -11, -5, -9, -11, 2, -3, 0, -14, -1, -13, -18, -12, 7, 17, 17, 14, 12, 10, 1, -10, -7, -8,
    -20, -19, -3, 3, 7, 21, 20, 13, 12, 8, 13, 20, 7, 1, 1, -1, 19, 4, -2, 0, 6, 5, -1, -6,
    -4, -7, -11, 2, -8, -5, -18, -31, -1, 8, 14, 12, 14, 14, 20, 25, 21, 2, -4, 11, 0, 17, 19, 0,
    21, 23, 10, 22, 31, 55, 36, 16, 11, 4, 39, 46, 47, 39, 51, 75, 75, 68, 20, 25, 21, 39, 75, 49,
    52, 58, 32, 63, 52, 45, 37, 52, 63, 48, 54, 50, 60, 69, 69, 59, 55, 50, 32, 31, 36, 9, 22, 14,
    2, 24, 26, 29, 5, 22, 24, 8, 21, 4, 13, 30, 25, 25, 40, 49, 41, 27, 9, -21, -26, -21, -10, 16,
    16, 3, 16, 26, 0, 0, -12, -23, -17, 0, -8, 1, 8, 2, 26, 12, 8, -10, 7, -6, -13, -1, -15, -21,
    -13, -42, -36, -57, -45, -45, -51, -44, -52, -52, -72, -76, -76, -85, -100, -85, -80, -92, -68, -68, -69, -47, -47, -64,
    -55, -47, -47, -30, -19, -18, -27, -20, -18, -26, -25, -37, -52, -44, -61, -61, -54, -67, -64, -53, -59, -56, -60, -55,
    -51, -36, -31, -39, -50, -50, -45, -14, -25, -7, -6, -31, -33, -54, -50, -63, -62, -61, -82, -55, -58, -59, -49, -49,
    -46, -36, -28, -24, -30, -23, -30, -12, -5, -22, -7, -19, -19, -20, -21, -26, -26, -28, -38, -35, -26, -26, -25, -23,
    -11, -10, -8, -12, -12, -13, -17, -8, 4, 9, 6, 0, 3, 8, 9, 21, 11, 9, 10, 14, 20, 13, 14, 14,
    13, 16, 16, 18, 14, 17, 26, 21, 40, 24, 13, 22, 26, 37, 36, 44, 45, 45, 50, 50, 49, 48, 59, 58,
    54, 64, 67, 81, 65, 65, 63, 57, 61, 49, 48, 62, 60, 56, 68, 62, 64, 62, 65, 63, 67, 62, 52, 56,
    54, 66, 66, 55, 56, 47, 59, 51, 45, 41, 31, 31, 40, 30, 33, 26, 22, 37, 45, 50, 36, 38, 30, 26,
    39, 44, 43, 47, 36, 46, 37, 37, 33, 40, 43, 32, 32, 28, 40, 39, 38, 42, 27, 41, 43, 52, 41, 39,
    29, 34, 50, 34, 31, 32, 23, 28, 19, 33, 19, 14, 9, -4, 14, 5, 3, -14, -9, -4, -11, 0, -17, -14,
    -18, -24, -17, -26, -22, -32, -33, -30, -38, -27, -36, -42, -40, -31, -23, -16, -8, -15, -18, -11, -22, -24, -27, -19,
    -15, -16, -6, -15, -19, -21, -31, -33, -20, -23, -36, -36, -36, -32, -19, -27, -28, -31, -34, -27, -28, -34, -31, -40,
    -39, -41, -54, -52, -51, -58, -61, -62, -58, -61, -57, -55, -53, -53, -52, -52, -57, -54, -55, -59, -61, -59, -50, -55,
    -47, -42, -56, -52, -54, -49, -51, -46, -58, -59, -53, -63, -59, -66, -67, -72, -74, -67, -59, -57, -64, -59, -51, -47,
    -38, -37, -44, -48, -42, -40, -42, -39, -41, -44, -40, -26, -37, -32, -35, -40, -37, -32, -31, -31, -20, -31, -24, -22,
    -25, -16, -14, -24, -16, -8, -8, 3, 3, -6, -8, -4, -5, -7, -6, -11, -15, -7, -14, -18, -14, -27, -24, -25,
    -19, -7, -19, -15, -19, -19, -6, -11, -7, -2, 0, 22, 26, 28, 25, 30, 27, 33, 43, 48, 39, 35, 37, 29,
    28, 34, 19, 22, 17, 22, 15, 30, 18, 20, 23, 21, 36, 32, 25, 16, 18, 16, 9, 20, 5, 8, 9, 4,
    4, -3, -6, -12, 0, -8, -9, -15, -15, -1, -11, -5, -1, -8, 8, 6, 13, 27, 37, 35, 34, 39, 35, 42,
    44, 33, 33, 33, 42, 42, 43, 38, 28, 35, 43, 44, 42, 36, 35, 37, 43, 34, 39, 31, 37, 43, 43, 40,
    30, 31, 21, 30, 27, 23, 29, 33, 46, 46, 57, 52, 49, 58, 51, 46, 46, 39, 41, 33, 35, 26, 28, 19,
    16, 17, 14, 19, 12, 7, 12, -7, -1, -1, 6, 6, 5, 13, 9, 17, 12, 6, 6, 2, 6, -1, 13, -8,
    -10, -4, -5, 1, -5, -10, -2, 8, 15, 18, 24, 7, 19, 21, 21, 24, 14, 13, 10, 8, 11, 17, 16, 7,
    5, 3, 4, -6, -17, -8, -24, -12, -11, -11, -1, -3, -5, -12, 13, -3, 0, 2, -14, -7, -12, -1, -9, -2,
    -6, -9, 7, -7, 5, -7, -6, -12, -17, -7, -16, -7, -12, -20, -22, -15, -22, -17, -10, -18, -14, -13, -15, -21,
    -10, -9, -12, -5, -14, -11, -17, -26, -31, -28, -29, -34, -30, -41, -23, -25, -18, -22, -35, -23, -38, -25, -34, -32,
    -22, -12, 7, 5, 20, 13, 20, 29, 22, 12, 2, -11, -10, -11, -20, -24, -25, -23, -31, -28, -39, -25, -16, -15,
    -2, -3, -7, -3, -21, -16, -20, -15, -4, -23, -17, -22, -3, -17, -16, -5, -15, -19, -15, -39, -27, -36, -29, -32,
    -23, -16, -21, -16, -7, -19, -1, -14, -20, -17, -8, -4, 3, 9, -5, 9, 2, -4, 11, 2, 6, 12, 10, 15,
    14, 18, -1, 5, 2, 0, 1, 1, -8, -3, -9, -13, -2, -6, -2, -5, 4, 5, 11, 7, 11, 14, 19, 25,
    9, 5, 1, 11, 1, -4, -8, -13, 6, 5, 3, 5, 13, 21, 18, 26, 23, 28, 30, 16, -2, 8, 20, 19,
    27, 31, 24, 22, 25, 29, 27, 19, -6, 2, 4, 8, 17, 6, 15, 20, 18, 11, 9, 29, 12, 20, 25, 40,
    28, 24, 25, 19, 38, 41, 21, 25, 30, 25, 24, 23, 25, 23, 21, 13, 17, 36, 33, 21, 23, 16, 32, 20,
    23, 18, 25, 24, 15, 18, 8, 2, -3, -14, -6, -3, -14, -24, -14, -12, -3, 7, -8, 8, 3, 13, 13, 11,
    7, 7, 7, 13, 21, 13, 20, 23, 19, 20, 9, -2, -6, -19, -30, -28, -27, -26, -20, -27, -20, -21, -19, -25,
    -18, -11, -22, -18, -25, -11, -16, -9, -17, -19, -5, -21, -15, -15, -16, -12, -18, -22, -26, -17, -22, -18, -20, -10,
    -6, -5, 2, -1, 6, 10, 11, 20, 12, 17, 20, 7, 9, 12, 12, 6, 15, 6, 4, 20, 18, 17, 19, 11,
    17, 23, 16, 13, 22, 8, 0, -11, -11, 11, 9, -1, -15, -3, 1, 4, -7, -9, -4, 3, 3, 2, 14, 10,
    -5, -6, -7, 2, 6, 5, -3, -4, 8, -2, -2, 3, 1, -10, -8, -7, -6, -2, -6, -16, 3, 14, 7, -3,
    21, 16, 15, 29, 12, 18, 27, 17, -4, 14, 2, -4, 6, -3, -4, 12, -25, -69, -25, -13, -8, 0, -40, -34,
    -2, -6, -17, 1, 8, -17, 12, -4, 4, 10, 16, 8, 20, 24, 15, 29, 19, 18, 18, 19, 33, 13, 4, -14,
    2, 5, 4, 7, 13, 2, -5, -12, -8, -1, -3, -3, -17, -5, 2, -12, -11, -13, -2, -8, -5, -2, 9, 2,
    8, -2, -7, 20, -17, -9, -11, 0, -4, -13, -7, -13, -11, -19, -21, -11, -23, -26, -27, -32, -29, -30, -31, -7,
    -17, -39, -24, -20, 3, -7, -12, -20, -6, -22, -22, 24, 49, 60, 33, 19, 44, 67, 38, 10, 24, 32, 36, 8,
    -7, 4, -6, -29, -32, -16, 1, 12, -9, -15, 13, 50, 47, 27, 35, 36, 35, 65, 75, 74, 74, 36, 51, 51,
    49, 14, 11, -1, -3, -13, -10, -28, -38, -35, -51, -50, -29, -45, -46, -45, -38, -41, -30, -47, -37, -30, -36, -26,
    -17, -17, -38, -35, -40, -28, -45, -54, -66, -59, -59, -58, -71, -43, -62, -55, -46, -43, -27, -30, -47, -39, -4, 1,
    6, 7, 12, 13, 14, 5, -8, 8, 24, 4, 16, 17, 35, 11, 2, -9, 4, 3, -18, -13, -10, -25, 2, -21,
    -24, -2, -20, -16, -4, 5, 1, 7, 2, 6, 8, 16, -7, 2, 7, 2, -1, -8, -10, -2, -20, -21, -20, -23,
    -21, -29, -42, -52, -36, -49, -55, -48, -35, -26, -14, -39, -25, -20, -28, -22, -30, -35, -30, -27, -44, -32, -22, -29,
    -30, -37, -34, -27, -27, -25, -53, -38, -32, -26, -34, -36, -17, 4, 13, -2, 18, 20, 33, 26, 35, 59, 46, 63,
    47, 38, 36, 42, 33, 31, 31, 17, 37, 51, 33, 28, 25, 33, 26, 45, 28, 21, 43, 34, 36, 37, 32, 28,
    41, 21, 35, 42, 37, 49, 43, 29, 49, 41, 43, 51, 39, 45, 43, 43, 43, 41, 24, 35, 24, 23, 36, 28,
    24, 49, 38, 30, 22, 30, 30, 37, 49, 39, 61, 44, 34, 28, 39, 43, 37, 27, 19, 33, 21, 18, 10, 13,
    34, 29, 7, 32, 28, 13, 20, 11, 18, 44, 34, 47, 56, 41, 51, 62, 52, 57, 67, 69, 76, 68, 63, 65,
    61, 75, 59, 50, 50, 42, 49, 50, 46, 39, 41, 35, 41, 33, 40, 60, 48, 52, 43, 40, 48, 30, 24, 32,
    21, 6, 8, 7, 6, 3, 0, -3, -3, -10, -12, -14, -12, -10, -12, -18, -29, -23, -22, -20, -18, -6, -19, -23,
    -41, -32, -39, -43, -48, -45, -49, -46, -60, -68, -53, -52, -66, -60, -59, -57, -52, -60, -57, -49, -43, -54, -55, -53,
    -46, -49, -54, -66, -69, -65, -77, -78, -70, -72, -72, -71, -80, -68, -68, -69, -68, -58, -63, -59, -57, -55, -50, -51,
    -54, -53, -52, -50, -59, -62, -64, -61, -60, -63, -79, -82, -94, -93, -78, -83, -75, -75, -82, -73, -59, -64, -50, -36,
    -35, -32, -37, -48, -46, -34, -44, -38, -42, -40, -35, -50, -41, -48, -51, -45, -53, -56, -56, -66, -64, -50, -55, -55,
    -50, -38, -17, -23, -16, -2, 5, 7, 10, 15, 16, 28, 16, 27, 21, 28, 34, 20, 42, 35, 28, 45, 38, 33,
    38, 40, 29, 31, 26, 16, 38, 32, 32, 39, 50, 41, 41, 43, 49, 62, 61, 54, 78, 78, 74, 82, 79, 90,
    97, 61, 59, 74, 60, 70, 73, 60, 90, 73, 51, 75, 49, 54, 62, 36, 45, 40, 33, 35, 37, 36, 50, 53,
    52, 49, 41, 47, 52, 47, 57, 35, 30, 33, 27, 23, 32, 26, 45, 58, 49, 68, 68, 74, 73, 65, 63, 56,
    38, 56, 46, 43, 65, 61, 41, 52, 39, 36, 38, 34, 33, 33, 21, 31, 19, 8, 11, 15, 7, 14, 11, 18,
    23, 22, 17, 3, -3, 0, 4, -5, -16, -13, 3, -5, -10, -11, -18, -13, -13, -38, -26, -19, -27, -16, -31, -20,
    -27, -39, -24, -41, -39, -33, -55, -33, -23, -30, -21, 2, -25, -17, -4, -15, 12, 21, 13, 7, -12, -4, 15, 7,
    12, 26, 24, 17, 5, -6, 2, -4, -12, -40, -33, -22, -49, -47, -43, -51, -54, -52, -54, -41, -58, -75, -56, -70,
    -63, -59, -71, -52, -42, -58, -49, -56, -66, -34, -56, -68, -40, -41, -48, -30, -49, -33, -25, -43, -32, -41, -62, -55,
    -57, -66, -74, -76, -90, -80, -60, -62, -73, -80, -79, -62, -59, -81, -60, -35, -46, -48, -31, -38, -23, -19, -33, -9,
    3, 4, -12, 9, 9, -5, -6, -7, -2, -1, -15, -25, -30, -20, -29, -44, -44, -47, -49, -52, -53, -43, -52, -48,
    -72, -61, -61, -68, -44, -49, -31, -44, -31, -24, -20, -14, -20, -4, -15, -11, -7, -18, -22, -18, -43, -25, -19, -39,
    -29, -27, -42, -29, -45, -30, -50, -47, -43, -49, -33, -45, -37, -24, -42, -40, -35, -29, -9, -18, -28, -13, -5, -11,
    -13, -10, 15, 25, 25, 37, 27, 10, 42, 45, 38, 51, 59, 45, 56, 40, 12, 34, 27, 10, 21, 4, 9, 18,
    34, 67, 90, 92, 96, 117, 123, 123, 118, 127, 127, 127, 120, 114, 116, 119, 101, 98, 77, 92, 80, 61, 58, 49,
    50, 41, 39, 54, 59, 72, 59, 66, 67, 63, 57, 62, 66, 69, 60, 67, 66, 88, 72, 69, 67, 79, 84, 62,
    73, 79, 69, 60, 55, 67, 58, 70, 42, 48, 52, 47, 45, 58, 52, 46, 49, 34, 34, 45, 36, 35, 26, 35,
    53, 27, 41, 46, 40, 51, 38, 45, 62, 41, 30, 47, 39, 34, 42, 21, 33, 34, 5, -5, 17, 31, 39, 56,
    -2, 9, 24, -7, 7, 9, 3, -16, -14, -11, -8, -6, -5, -11, -11, -4, -19, 10, 15, 23, 34, 23, 20, 26,
    40, 45, 27, 36, 18, 0, 7, -7, -9, -8, -12, -22, -30, -44, -46, -34, -49, -55, -61, -64, -74, -87, -64, -57,
    -62, -54, -58, -41, -55, -51, -62, -59, -51, -82, -80, -72, -85, -66, -74, -73, -60, -49, -44, -67, -49, -53, -60, -50,
    -71, -54, -63, -74, -73, -71, -90, -89, -82, -99, -94, -109, -127, -122, -127, -127, -124, -127, -127, -112, -124, -115, -102, -114,
    -123, -109, -91, -80, -57, -64, -55, -38, -57, -14, -18, -12, -3, -24, 11, -9, -17, 0, -16, 14, -27, -12, -11, -41,
    -9, -39, -25, -38, -62, -34, -53, -40, -39, -53, -48, -46, -34, -17, -10, -5, -2, -15, -3, -5, 1, 17, 14, 10,
    8, 23, 22, 27, 27, 7, 23, 2, 4, 10, 1, 7, -17, -11, -13, -25, -16, -37, -31, -38, -29, -19, -42, -19,
    -26, -32, -15, -35, -13, -1, -9, -13, 6, 18, 14, 15, 14, 19, 20, 14, 27, 37, 36, 27, 40, 38, 45, 49,
    38, 53, 48, 34, 38, 18, 29, 23, 9, 28, 24, 30, 29, 36, 55, 43, 56, 42, 47, 60, 60, 69, 73, 66,
    74, 86, 86, 83, 89, 81, 75, 74, 69, 79, 60, 72, 61, 47, 40, 31, 47, 30, 22, 32, 14, 12, 2, 18,
    28, 12, 13, 18, 29, 32, 32, 44, 42, 42, 29, 35, 41, 44, 60, 54, 47, 52, 46, 47, 59, 55, 51, 36,
    21, 16, 27, 23, 26, 32, 32, 35, 37, 35, 47, 48, 37, 38, 31, 30, 27, 36, 38, 26, 17, 29, 28, 23,
    43, 36, 36, 35, 16, 22, 26, 23, 26, 8, 23, 21, 12, 25, 17, 16, 7, 9, -2, -4, 7, -16, -2, -10,
    -25, -15, -20, -8, -4, -20, -10, -14, 1, 4, -10, 15, -15, -9, 3, -6, 21, 8, 11, 11, 9, -16, -5, 1,
    -16, 1, -28, -35, -13, -39, -25, -20, -38, -20, -25, -19, -3, -10, 3, -4, 4, -12, 0, 13, 10, 28, 26, 21,
    32, 37, 33, 32, 22, 11, 20, 10, 11, 11, -2, -5, -13, -18, -25, -23, -27, -41, -41, -39, -53, -30, -29, -44,
    -16, -29, -41, -25, -35, -20, -8, 10, 2, 3, -3, -13, 5, 3, 0, -3, -19, -23, -14, -16, -20, -24, -19, -25,
    -22, -17, -26, -2, -3, 2, 17, 4, 5, 36, 34, 12, 22, 16, 13, 20, 12, 5, 4, -17, -19, 0, -8, -8,
    -16, -9, -18, -33, -34, -35, -35, -53, -42, -44, -49, -51, -52, -39, -41, -55, -52, -48, -47, -48, -36, -28, -26, -35,
    -36, -32, -25, -32, -16, -16, -9, -5, -5, -7, -21, -18, -20, -3, -5, -2, 2, 15, 16, 11, 25, 20, 28, 24,
    33, 25, 27, 55, 30, 35, 23, 10, 26, 10, 8, -1, -4, -15, -31, -25, -36, -36, -34, -33, -37, -42, -39, -44,
    -32, -26, -26, -18, -21, -21, 6, -8, -17, -3, -1, 2, 3, 2, 2, 17, 4, 8, 12, -18, -16, -14, -15, -11,
    -21, -11, -10, -28, -27, -33, -20, -24, -48, -38, -28, -36, -28, -21, -27, -23, -26, -25, -17, -21, -9, -4, 3, 7,
    2, 0, -6, -6, 0, 20, 17, -1, 23, 16, 9, 16, 5, 24, 11, 0, 3, 4, 7, 5, 1, -8, -8, -13,
    -24, -7, -24, -33, -14, -13, -2, 1, -1, 11, 16, 7, 33, 15, 32, 14, 12, 44, 27, 38, 40, 34, 39, 32,
    31, 28, 4, 12, -7, -17, -7, -25, -29, -20, -34, -35, -37, -38, -38, -23, -26, -9, -7, -14, 6, 3, -13, 20,
    37, 52, 91, 68, 69, 74, 53, 56, 62, 63, 47, 36, 30, 22, 34, 17, 24, 26, 16, 11, -3, 9, 15, 17,
    11, 13, -3, 8, 9, 7, 14, -12, 10, 8, 4, 14, 19, 32, 48, 48, 25, 37, 29, 38, 53, 32, 57, 24,
    23, 31, 1, 42, 11, 4, 13, -9, 3, 7, -9, -11, -11, -26, -22, -21, -19, -42, -33, -43, -39, -16, -25, -27,
    -25, -33, -22, -16, -12, 5, -23, -18, -19, -27, -12, 4, -4, 8, 10, 9, 34, 23, 15, 24, 20, 18, 12, 4,
    3, -15, -8, -5, -15, -18, -36, -28, -16, -29, -20, -23, -42, -49, -45, -56, -51, -33, -30, -24, -15, -14, -10, 0,
    13, 13, 14, 16, 6, 29, 23, 23, 35, 31, 24, 11, -12, -7, -12, -28, 4, -17, -21, -17, -30, -9, -6, -20,
    13, 23, 28, 37, 28, 46, 54, 45, 53, 49, 32, 46, 40, 51, 35, 11, 14, 26, 26, 13, 28, 10, -6, -5,
    -16, 20, 28, 12, 21, 8, 18, 25, 24, 24, 35, 15, 16, 28, 10, 37, 20, 11, 0, -16, -6, -16, -26, -31,
    -45, -34, -50, -53, -39, -35, -40, -39, -36, -31, -19, -2, -6, 5, 11, -5, 37, 10, 2, 33, 14, 22, 25, -1,
    22, 17, -6, 1, -9, -17, -3, -24, -11, -15, -17, -27, -29, -7, -9, -6, -4, -19, -18, -7, 1, 9, 4, -2,
    -13, -9, -22, -26, -9, -17, -33, -20, -33, -27, -20, -38, -28, -12, -28, -22, -24, -25, -12, -29, -13, -25, -26, -20,
    -35, -13, -33, -10, 5, -17, 6, -5, -8, 2, -18, -8, 4, -10, -18, -3, -23, -16, -17, -27, -9, -16, -28, -20,
    -27, -32, -18, -29, -43, -44, -62, -56, -48, -69, -61, -60, -68, -43, -21, -19, 4, 2, 9, 6, 15, 18, 11, 21,
    -1, 10, 6, -2, -6, 4, -2, -14, 10, -6, -10, -6, -14, -27, -24, -21, -35, -9, -21, -25, -6, 5, 9, 19,
    21, 5, 14, 18, 15, 27, 26, 9, 4, -5, -32, 19, -5, 11, 42, -7, 29, 10, 31, 36, 48, 53, 24, 36,
    -1, 5, 23, 1, 11, -2, -8, -8, -17, -11, -9, -1, -14, -16, -20, -31, -19, -12, -19, -12, -15, -11, -14, -14,
    -16, -7, 33, 13, 52, 46, 26, 46, 19, 24, 42, 70, 42, 42, 36, 11, 38, 37, 31, 39, 24, 6, 17, 26,
    26, 40, 33, 23, 20, 23, 26, 26, 24, -6, 9, -17, -12, -17, -36, -26, -57, -64, -55, -59, -58, -60, -72, -64,
    -57, -31, -40, -19, -18, -38, -7, -22, -8, 27, 14, 33, 20, 28, 37, 41, 45, 27, 22, 13, 4, 2, -10, -15,
    -28, -40, -43, -32, -31, -28, -35, -35, -36, -25, -5, -6, -3, 5, -6, -15, 6, -8, 1, 32, 7, 17, 22, 5,
    23, 26, 17, 22, 32, 27, 27, 39, 22, 40, 14, 21, 41, 44, 57, 35, 38, 32, 35, 46, 32, 47, 26, 13,
    18, 1, 22, 9, 8, 8, -1, 16, 10, 15, 14, 2, 0, 7, 3, 2, 1, 0, -10, -11, 5, 10, 14, 23,
    11, 12, 15, 21, 16, 36, 41, 21, 33, 28, 24, 32, 29, 25, 23, 8, 23, 28, 42, 43, 29, 31, 20, 21,
    39, 48, 48, 53, 38, 35, 48, 40, 51, 50, 33, 31, 13, 25, 35, 22, 35, 18, 19, 5, -2, -2, -19, -12,
    -35, -41, -38, -45, -47, -41, -56, -43, -39, -41, -30, -29, -27, -30, -30, -15, -19, -21, -10, -47, -16, -19, -35, -10,
    -10, -16, -1, -8, -47, -33, -32, -40, -18, -34, -49, -48, -59, -47, -40, -21, -31, -18, -26, -29, -5, -21, -4, 8,
    -1, 3, -6, -18, -8, -8, -23, 2, -12, -16, -3, -16, -18, -19, -18, -24, -24, -34, -41, -35, -63, -41, -42, -51,
    -7, -39, -45, -27, -30, -28, 0, 0, 7, 12, -4, -10, 8, 5, 12, 27, 16, 10, 32, 12, 28, 48, 20, 43,
    48, 28, 56, 59, 41, 83, 51, 48, 86, 71, 68, 69, 57, 48, 65, 54, 42, 44, 40, 45, 17, 20, 21, 5,
    26, 22, 13, 33, 33, 15, 14, 20, 7, 7, -7, -19, -2, -11, 3, 0, -3, 16, 6, 8, 23, 8, 24, 16,
    -17, -6, -3, -1, 10, 11, 18, 15, -3, 11, 13, 21, 42, 23, 35, 28, 29, 21, 22, 26, 20, 32, 12, 13,
    17, -11, 22, 0, -1, -2, -18, -18, -28, -41, -24, -19, -51, -10, -47, -31, -16, -62, -41, -60, -62, -27, -66, -63,
    -53, -80, -25, -25, -25, 9, -22, -7, -3, 9, 52, 29, 12, 3, -13, -28, -21, 2, -47, -51, -51, -70, -60, -74,
    -103, -89, -111, -127, -117, -121, -120, -127, -120, -127, -127, -126, -122, -94, -102, -97, -77, -65, -51, -42, -41, -42, -31, -38,
    -29, -11, -27, -11, -39, -37, -32, -25, 3, -3, -3, -36, -31, -12, -9, 19, 48, 22, 24, 38, 12, 33, 44, 10,
    34, 15, 15, 6, -7, -7, -20, -35, -39, -53, -56, -62, -68, -64, -71, -81, -51, -78, -79, -72, -83, -60, -50, -36,
    -43, -25, -18, -29, 1, -2, 25, 31, 21, 42, 18, 28, 2, 5, 23, 10, 8, -9, -9, -15, -7, 9, -10, 17,
    17, -4, 24, 11, 18, 71, 58, 38, 45, 19, 35, 48, 30, 54, 61, 34, 27, 51, 41, 78, 77, 66, 91, 80,
    98, 98, 87, 116, 127, 120, 122, 111, 118, 111, 108, 127, 112, 127, 127, 118, 127, 109, 124, 127, 105, 124, 107, 84,
    100, 58, 50, 80, 36, 82, 69, 49, 80, 38, 59, 57, 38, 65, 30, 26, 21, 3, 32, 9, 31, 30, 23, 32,
    7, 27, 37, 29, 51, 49, 15, 34, 16, -5, 1, 8, 3, 6, 8, -12, 24, 17, 9, 25, 11, 6, 18, -21,
    3, -6, -27, -11, -26, -23, -29, -28, -23, -21, -1, -7, -11, -8, -10, -8, -2, -13, -15, 8, 1, 11, 17, 18,
    21, 27, 21, 6, 21, -5, -1, 10, -29, -12, -36, -53, -20, -51, -28, -50, -57, -48, -58, -44, -55, -50, -47, -50,
    -49, -56, -60, -69, -51, -59, -48, -41, -55, -40, -64, -68, -34, -52, -38, -44, -64, -45, -61, -82, -58, -51, -75, -63,
    -68, -81, -72, -83, -76, -50, -74, -76, -66, -54, -61, -51, -40, -24, -27, -30, -1, -8, -16, -15, -19, -10, 0, -28,
    1, -8, -17, -25, -20, -16, -19, -26, -55, -29, -53, -25, -34, -26, -23, -47, -7, -18, -9, -22, -25, 3, -3, 6,
    7, -9, 7, -10, 7, 26, 38, 23, 43, 36, 35, 82, 64, 64, 93, 56, 56, 66, 45, 42, 60, 31, 37, 35,
    5, 47, 10, 15, 24, 9, 12, 17, 5, 4, 13, 2, 10, -11, -12, 30, -29, 26, 10, 8, 42, 14, 35, 47,
    48, 62, 77, 67, 64, 89, 71, 95, 74, 68, 89, 55, 90, 78, 59, 73, 60, 40, 67, 69, 33, 53, 32, 19,
    34, 2, 16, -11, -20, -18, -30, -6, -16, -14, -8, 0, 5, 33, 26, 33, 63, 58, 32, 45, 45, 33, 19, 17,
    23, 26, 43, 9, 8, 22, 11, 11, 7, 12, -1, -9, -19, -10, -23, -3, -35, -1, 4, -42, 19, -38, -1, 4,
    -20, 1, 6, -10, -19, -10, -24, 12, -5, -10, 8, 4, -1, 7, 18, -1, 13, -13, -20, -17, -56, -42, -48, -42,
    -58, -81, -75, -83, -100, -85, -100, -86, -95, -88, -84, -88, -60, -66, -45, -49, -42, -27, -10, -13, -12, 24, 10, 7,
    5, 18, -6, 13, -2, -16, -1, -36, -29, -30, -49, -62, -62, -70, -92, -97, -96, -90, -79, -103, -106, -85, -107, -87,
    -89, -94, -69, -82, -79, -55, -76, -49, -28, -64, -37, -37, -36, -10, -18, -10, 16, -11, 2, 9, 6, 12, 8, -18,
    14, -13, -21, -9, -22, -11, -37, -11, -22, -27, -9, -9, -2, 12, 4, 1, 14, 11, 2, 15, -4, -2, 0, -14,
    2, 1, 7, 14, 7, 9, 15, 3, 0, 20, 2, 11, 5, -26, -3, -14, 7, 12, 6, 9, 21, 39, 41, 56,
    82, 74, 99, 106, 108, 127, 119, 127, 127, 118, 127, 127, 127, 113, 113, 120, 127, 124, 101, 122, 103, 97, 101, 115,
    91, 84, 64, 102, 92, 73, 96, 102, 127, 120, 112, 127, 127, 125, 127, 111, 127, 91, 85, 88, 52, 59, 39, 38,
    53, 14, 37, 25, 36, 35, 26, 47, 13, 21, 52, 16, 46, 18, 23, 33, 17, 4, 8, -5, -23, -42, -55, -56,
    -63, -70, -62, -37, -90, -51, -47, -46, -24, -66, -50, -52, -32, -38, -34, -10, -39, 0, -26, -1, 16, 3, 36, 11,
    17, -1, 2, 7, -17, 4, -15, -29, -38, -36, -21, -47, -49, -52, -30, -55, -62, -67, -81, -57, -112, -92, -71, -102,
    -68, -89, -94, -67, -76, -55, -45, -71, -42, -46, -25, -1, -35, -21, -48, -27, 11, -6, -9, -18, -7, -13, -7, -24,
    -40, -31, -50, -45, -70, -70, -72, -77, -84, -95, -100, -85, -95, -75, -75, -98, -74, -66, -79, -57, -50, -54, -46, -41,
    -45, -17, -41, -31, -39, -10, -25, -31, 8, -13, 24, 2, 14, -4, 6, 2, -27, 19, -45, -19, -12, -48, -7, -46,
    -24, 3, -14, -18, -3, -6, -10, 0, -28, 7, -10, -25, -13, -16, -11, -22, 0, -31, -13, -21, -28, 3, -48, -27,
    -34, -41, -40, -58, -20, -61, -49, -59, -69, -42, -81, -56, -64, -78, -59, -80, -46, -52, -64, -56, -63, -58, -58, -59,
    -55, -61, -68, -74, -48, -49, -50, -51, -47, -45, -54, -46, -38, -50, -50, -32, -43, -21, -32, -31, -27, -24, -13, -13,
    11, 11, 43, 18, 29, 47, 54, 72, 46, 83, 72, 83, 86, 81, 95, 90, 94, 77, 93, 97, 91, 82, 97, 106,
    109, 103, 98, 117, 127, 103, 117, 119, 122, 127, 104, 120, 125, 89, 121, 83, 85, 99, 63, 72, 68, 59, 55, 61,
    46, 56, 51, 34, 42, 52, 27, 34, 16, 39, 36, 34, 29, 29, 47, 25, 33, 22, 47, 41, 32, 45, 34, 48,
    42, 27, 54, 45, 37, 63, 31, 72, 37, 56, 55, 53, 66, 54, 67, 62, 57, 53, 51, 59, 41, 54, 31, 25,
    41, 26, 25, 29, 33, 34, 34, 19, 53, 52, 42, 55, 23, 82, 79, 73, 78, 64, 75, 64, 46, -2, 6, -6,
    -24, -13, -37, -40, -36, -64, -54, -59, -66, -46, -40, -37, -43, -36, -73, -44, -60, -46, -46, -83, -48, -77, -68, -68,
    -45, -42, -78, -71, -79, -92, -100, -122, -74, -70, -72, -85, -69, -68, -42, -54, -57, 0, -23, -1, 3, 3, 20, 14,
    4, 30, 23, 0, 4, -14, -16, -20, -30, -6, -26, -40, -42, -52, -51, -60, -74, -61, -66, -67, -57, -59, -46, -43,
    -53, -29, -35, -42, -47, -36, -21, -17, -16, -21, 5, 7, -1, 11, 33, 27, 26, 39, 31, 39, 28, 1, 40, 30,
    26, 17, -5, 22, -1, -2, -9, -18, -26, -29, -43, -46, -42, -44, -29, -38, -36, -32, -26, -20, -29, -25, -28, -17,
    -24, 6, -14, 17, 37, 8, 34, 28, 33, 66, 63, 60, 73, 79, 70, 97, 80, 84, 114, 57, 79, 72, 61, 90,
    73, 75, 74, 49, 50, 47, 33, 48, 42, 20, 29, 34, 29, 38, 19, 26, 31, 11, 33, 26, 45, 49, 26, 25,
    30, 26, 25, 28, 28, 32, 28, 11, 30, 13, 13, 21, 7, -16, -17, -18, -41, -21, -26, -25, -16, -42, -27, -27,
    -49, -42, -28, -54, -40, -47, -48, -45, -53, -71, -38, -63, -57, -39, -77, -52, -61, -74, -53, -83, -71, -62, -72, -88,
    -95, -93, -101, -91, -93, -94, -94, -79, -89, -90, -87, -93, -80, -92, -97, -52, -76, -68, -69, -74, -57, -62, -61, -63,
    -55, -73, -65, -80, -85, -51, -85, -83, -85, -87, -79, -107, -90, -69, -72, -76, -67, -62, -64, -67, -55, -38, -47, -51,
    -35, -26, -19, -42, -43, -18, -27, -35, -24, -51, -14, -26, -49, -9, -25, -15, -18, -27, -19, -6, -20, -15, 7, -16,
    -1, 1, -14, 9, -4, -10, -7, 7, 0, -3, 12, -3, 36, 3, -8, 28, 22, 29, 2, 22, 37, 29, 31, 21,
    54, 48, 21, 44, 47, 49, 46, 22, 34, 63, 39, 37, 53, 64, 78, 66, 41, 62, 74, 47, 60, 65, 83, 63,
    60, 49, 89, 69, 44, 61, 64, 76, 58, 60, 76, 72, 49, 55, 60, 75, 48, 38, 47, 36, 52, 31, 31, 63,
    35, 31, 28, 28, 37, 32, 20, 27, 29, 12, 2, 23, 23, 36, 18, 19, 29, 33, 31, 14, 32, 12, 9, 13,
    -17, 11, -8, -9, 13, -29, 7, -19, -7, 6, 8, -3, 8, 7, 2, 2, -23, 8, -14, -25, -15, -40, 0, -32,
    -12, -5, -11, 18, -6, 27, 9, 18, 31, -3, 15, -3, 8, -5, -4, 6, -6, 13, -2, 9, 18, -3, 19, 2,
    1, 4, 2, -2, -2, 7, -4, 10, -18, -5, -4, -19, -4, 4, -9, -5, -14, -13, -1, -35, -15, -18, -33, -29,
    -43, -34, -32, -39, -50, -45, -44, -59, -31, -59, -32, -36, -56, -9, -43, -12, -27, -54, -21, -30, -26, -14, -16, -15,
    -5, -23, 6, 15, 3, 22, 19, 25, 27, 41, 45, 45, 42, 31, 36, 31, 41, 45, 46, 39, 51, 43, 58, 41,
    58, 44, 30, 44, 34, 31, 34, 32, 36, 27, 45, 37, 47, 50, 60, 60, 43, 66, 60, 75, 60, 68, 67, 59,
    71, 56, 75, 46, 72, 72, 65, 78, 56, 74, 60, 57, 59, 41, 52, 38, 32, 13, 15, 20, 14, 10, 10, 34,
    22, 16, 25, 6, 28, 6, 11, 7, 5, 6, 17, -20, -8, -13, -29, -11, -26, -24, -24, -50, -38, -52, -68, -65,
    -52, -73, -62, -69, -88, -64, -99, -73, -74, -88, -86, -94, -100, -101, -89, -100, -90, -106, -79, -74, -73, -59, -73, -55,
    -53, -60, -48, -44, -52, -32, -53, -32, -41, -45, -27, -51, -28, -32, -34, -15, -44, -20, -33, -31, -23, -38, -39, -39,
    -32, -59, -29, -47, -34, -29, -43, -34, -38, -37, -47, -25, -39, -21, -4, -17, 14, -10, 30, 19, 11, 34, 11, 32,
    29, 28, 31, 43, 34, 45, 46, 39, 57, 47, 44, 69, 63, 66, 67, 47, 59, 62, 55, 75, 49, 45, 46, 42,
    41, 44, 39, 46, 39, 50, 65, 57, 69, 71, 62, 74, 72, 97, 87, 85, 80, 90, 87, 84, 79, 76, 64, 47,
    36, 33, 12, 29, 18, 0, 4, 7, 13, 6, 5, 11, 14, 17, 22, 14, 49, 35, 66, 69, 71, 94, 79, 85,
    71, 80, 69, 58, 61, 29, 42, 32, 23, 29, 6, 19, 22, 13, 24, 13, 23, 35, 15, 30, 40, 25, 57, 45,
    42, 64, 28, 46, 22, 16, 27, 21, 4, 3, -3, -9, -17, -23, -36, -33, -56, -63, -61, -62, -76, -62, -62, -50,
    -46, -51, -51, -72, -55, -68, -68, -64, -78, -77, -73, -73, -66, -63, -71, -52, -59, -50, -48, -42, -44, -45, -38, -55,
    -43, -58, -65, -57, -61, -52, -48, -51, -38, -41, -22, -28, -19, -31, -28, -37, -43, -34, -43, -34, -45, -57, -54, -60,
    -59, -58, -59, -76, -61, -73, -63, -61, -72, -42, -48, -46, -44, -48, -23, -31, -20, -9, -15, -15, -5, -12, -8, 2,
    -1, 2, 0, -2, 18, 3, 7, 18, 10, 19, 21, 23, 41, 31, 41, 48, 43, 44, 46, 43, 32, 37, 15, 39,
    27, 29, 14, 13, 14, 6, 11, -11, 8, 8, 8, 14, 24, 25, 40, 33, 24, 58, 28, 39, 45, 49, 50, 66,
    49, 67, 73, 63, 83, 72, 76, 71, 80, 83, 87, 74, 65, 65, 52, 57, 55, 49, 53, 45, 39, 31, 42, 32,
    38, 28, 29, 31, 8, 19, 16, 22, 37, 12, 44, 24, 16, 18, -4, 24, 16, 19, 34, 23, 15, 4, 9, 5,
    1, -10, -9, -15, -20, -12, -14, -12, -15, -29, -36, -44, -52, -43, -64, -66, -74, -76, -78, -60, -61, -57, -51, -55,
    -48, -44, -48, -18, -36, -39, -23, -33, -18, -29, -26, -8, -14, 6, 5, 16, 15, 26, 20, 22, 19, 4, 15, -4,
    -3, -7, -7, -9, -24, -24, -38, -34, -37, -46, -48, -36, -28, -19, -21, -21, -13, -10, -7, -12, 6, -16, 12, -8,
    0, -4, -11, -12, -17, -13, -19, -27, -27, -40, -27, -30, -23, -21, -38, -28, -39, -29, -28, -40, -18, -26, -14, -17,
    -3, -15, 0, -5, -2, 13, 17, 13, 35, 20, 37, 29, 35, 19, 37, 20, 30, 18, 2, 5, -11, 0, 2, -5,
    -13, -15, -23, -2, -1, -8, 13, 4, 9, 25, 0, 28, 30, 18, 47, 35, 38, 53, 45, 51, 68, 56, 72, 80,
    70, 78, 69, 63, 78, 65, 60, 55, 40, 34, 31, 31, 29, 31, 13, 24, 24, 32, 37, 13, 23, 9, 16, 5,
    19, 7, 7, 13, -9, -3, -19, -13, -12, -16, -19, -28, -24, -36, -27, -29, -26, -22, -44, -40, -47, -65, -51, -70,
    -62, -69, -65, -61, -60, -65, -57, -70, -58, -55, -65, -57, -81, -70, -66, -86, -76, -83, -91, -81, -98, -92, -83, -83,
    -83, -91, -84, -77, -55, -60, -44, -27, -52, -30, -37, -18, -14, -17, -26, -17, -29, -29, -27, -39, -25, -34, -22, -26,
    -37, -25, -26, -20, -18, -23, -2, -12, -13, 7, -14, 13, -2, 9, 12, 5, 28, 27, 26, 27, 29, 34, 27, 20,
    32, 39, 27, 30, 8, 13, 21, 15, 15, 17, 23, 29, 23, 16, 25, 42, 28, 27, 39, 32, 41, 27, 17, 36,
    28, 39, 40, 38, 41, 42, 43, 47, 46, 55, 55, 61, 64, 57, 60, 63, 66, 56, 60, 61, 72, 74, 67, 77,
    86, 95, 95, 89, 103, 100, 88, 88, 80, 88, 70, 69, 64, 51, 52, 52, 40, 46, 41, 36, 46, 30, 39, 25,
    36, 18, 15, 14, 12, -1, -4, -5, -4, 2, -6, -4, -5, -6, -5, -10, -20, -22, -26, -29, -25, -33, -22, -31,
    -22, -42, -37, -39, -37, -37, -39, -30, -19, -17, -13, -15, -10, -19, -26, -16, -29, -20, -24, -44, -30, -32, -32, -37,
    -38, -41, -21, -29, -37, -21, -41, -23, -27, -38, -20, -27, -20, -15, -16, -21, 5, -10, -8, -4, -5, -4, -12, -13,
    -13, -9, -6, -15, -14, -26, -28, -20, -36, -30, -46, -49, -51, -70, -54, -61, -63, -80, -75, -73, -68, -75, -72, -83,
    -82, -68, -70, -73, -76, -68, -72, -81, -70, -81, -74, -69, -71, -61, -72, -73, -65, -60, -51, -49, -46, -44, -51, -25,
    -28, -13, -17, -18, 9, 1, 18, 21, 24, 19, 18, 7, 18, 12, 2, -7, -7, -6, -6, -1, -17, -14, -11, -26,
    -16, -23, -20, -18, -20, -15, -14, -13, -11, -3, -7, 4, 2, -11, -3, -7, -14, -6, 0, -11, -12, -7, -15, 3,
    -13, -18, -9, -16, -4, -12, -24, -5, -16, -17, -15, -12, -13, -9, -12, -9, 3, -8, -9, -5, -10, 0, -3, -3,
    -3, 8, 14, 18, 24, 20, 22, 21, 26, 30, 19, 22, 29, 36, 35, 44, 47, 54, 57, 59, 71, 70, 76, 71,
    76, 86, 78, 83, 77, 79, 74, 72, 72, 62, 65, 69, 63, 63, 66, 62, 61, 57, 72, 69, 55, 62, 55, 64,
    66, 70, 73, 67, 71, 65, 77, 72, 66, 78, 65, 77, 68, 59, 63, 69, 69, 70, 80, 68, 77, 71, 80, 85,
    93, 86, 92, 81, 86, 93, 87, 90, 85, 78, 83, 68, 72, 66, 72, 64, 60, 46, 46, 39, 39, 32, 22, 27,
    26, 28, 11, 9, 11, 17, 7, -1, -5, 0, 2, 3, 2, 1, 6, -10, -18, -24, -24, -18, -28, -28, -33, -39,
    -52, -52, -57, -44, -54, -71, -84, -86, -78, -78, -86, -89, -91, -78, -87, -88, -96, -88, -92, -102, -100, -105, -101, -104,
    -98, -105, -108, -111, -106, -115, -108, -101, -108, -110, -115, -114, -114, -112, -118, -116, -118, -104, -106, -101, -98, -99, -83, -89,
    -85, -76, -70, -73, -94, -81, -80, -74, -71, -66, -67, -67, -67, -67, -64, -69, -56, -65, -60, -63, -48, -52, -47, -30,
    -41, -33, -35, -16, -10, -18, -11, -11, -8, -3, 2, 11, 3, 0, -1, 13, 8, 10, 19, 21, 22, 16, 9, 24,
    20, 26, 27, 18, 25, 25, 25, 37, 39, 40, 42, 36, 38, 46, 37, 49, 52, 60, 56, 47, 61, 63, 66, 72,
    73, 72, 80, 72, 86, 88, 85, 78, 80, 103, 85, 75, 81, 77, 79, 81, 77, 79, 84, 77, 73, 74, 84, 76,
    77, 97, 90, 91, 89, 89, 95, 105, 106, 95, 96, 99, 92, 92, 105, 97, 97, 88, 100, 80, 92, 96, 86, 96,
    70, 69, 71, 72, 73, 60, 63, 60, 59, 43, 51, 42, 50, 36, 35, 20, 25, 13, 3, 12, -2, 7, 2, 2,
    -8, -18, -13, -16, -32, -16, -28, -29, -37, -47, -39, -41, -36, -38, -57, -50, -51, -57, -54, -59, -57, -65, -70, -79,
    -76, -69, -83, -89, -94, -89, -98, -93, -107, -101, -107, -116, -116, -127, -113, -114, -114, -110, -116, -107, -106, -115, -105, -105,
    -101, -107, -103, -100, -93, -95, -105, -105, -100, -86, -92, -82, -85, -80, -87, -91, -75, -79, -75, -74, -87, -78, -78, -68,
    -70, -58, -55, -53, -36, -53, -42, -35, -28, -38, -36, -26, -19, -18, -13, -6, -16, -12, 2, -3, -7, -7, 24, 3,
    16, 8, 19, 21, 7, 25, 26, 31, 22, 22, 31, 27, 23, 32, 41, 40, 30, 41, 36, 42, 48, 59, 57, 66,
    73, 80, 70, 64, 72, 74, 86, 95, 88, 87, 79, 92, 95, 107, 105, 111, 112, 119, 118, 109, 108, 98, 97, 90,
    100, 94, 95, 87, 76, 75, 67, 90, 85, 84, 99, 91, 90, 85, 77, 80, 83, 83, 74, 65, 64, 56, 52, 43,
    47, 58, 48, 50, 46, 49, 50, 32, 29, 43, 24, 33, 20, 20, 15, 12, 16, 1, 13, 13, 15, 12, 11, 18,
    7, 6, 2, 1, 1, -3, -9, -13, -18, -15, -10, -17, -16, -12, -15, -8, -19, -13, -16, -21, -24, -38, -41, -39,
    -46, -49, -59, -61, -64, -65, -72, -72, -66, -74, -53, -70, -72, -84, -73, -86, -85, -89, -100, -102, -122, -111, -104, -103,
    -105, -105, -108, -103, -96, -80, -92, -87, -84, -83, -77, -87, -69, -81, -81, -85, -77, -71, -69, -63, -70, -67, -68, -61,
    -57, -50, -45, -50, -47, -48, -43, -44, -48, -42, -41, -32, -44, -40, -27, -28, -13, -24, -23, -15, -15, -7, -13, -5,
    -11, -10, -10, -12, -8, -4, 1, -7, 0, 6, 20, 16, 5, 20, 29, 28, 20, 28, 36, 26, 29, 29, 40, 44,
    56, 41, 56, 57, 64, 63, 57, 69, 62, 66, 72, 59, 85, 77, 82, 67, 73, 72, 79, 93, 88, 83, 83, 79,
    76, 80, 89, 79, 77, 76, 82, 76, 63, 68, 66, 72, 64, 60, 67, 64, 70, 55, 71, 61, 61, 61, 52, 55,
    52, 65, 61, 51, 51, 47, 54, 55, 54, 55, 54, 43, 48, 36, 33, 35, 28, 31, 27, 21, 17, 21, 17, 10,
    21, 10, 8, 6, 11, -2, 6, 3, -4, -3, -4, -11, -9, -1, -8, -10, -16, -21, -29, -17, -12, -28, -23, -28,
    -31, -39, -47, -32, -39, -41, -58, -56, -64, -60, -67, -66, -77, -71, -65, -77, -72, -67, -75, -70, -83, -76, -71, -72,
    -83, -77, -72, -74, -66, -75, -73, -63, -63, -47, -60, -60, -64, -49, -49, -57, -45, -61, -50, -53, -56, -46, -48, -47,
    -58, -52, -41, -47, -40, -54, -39, -41, -40, -28, -43, -31, -27, -32, -25, -29, -24, -26, -22, -20, -14, -20, -20, -17,
    -6, -8, -9, -1, -12, -4, 8, 2, 4, 0, -13, -3, 6, 15, -9, 5, -2, 20, 15, 21, 31, 36, 27, 28,
    34, 44, 40, 33, 39, 43, 46, 37, 47, 42, 62, 67, 67, 69, 77, 88, 85, 80, 83, 100, 92, 80, 95, 90,
    84, 81, 89, 94, 87, 75, 94, 85, 90, 93, 95, 87, 88, 79, 84, 75, 79, 75, 72, 71, 64, 78, 69, 76,
    68, 55, 64, 55, 67, 49, 64, 47, 53, 59, 40, 42, 32, 23, 23, 33, 30, 46, 45, 34, 33, 48, 45, 47,
    42, 62, 39, 41, 38, 38, 30, 31, 35, 30, 32, 41, 34, 23, 19, 29, 29, 17, 13, 16, 21, -6, -6, 2,
    -6, -11, -19, -13, -9, -7, 2, -13, -17, -29, -9, -17, -15, -15, -18, -21, -42, -45, -52, -48, -54, -57, -63, -63,
    -63, -66, -82, -77, -66, -73, -70, -71, -68, -71, -78, -76, -79, -72, -83, -69, -61, -72, -68, -70, -66, -81, -62, -55,
    -61, -67, -66, -73, -75, -82, -75, -82, -73, -74, -91, -79, -81, -81, -77, -84, -75, -89, -86, -79, -78, -81, -81, -80,
    -78, -81, -85, -82, -83, -79, -82, -87, -93, -94, -82, -73, -84, -84, -84, -88, -94, -94, -81, -74, -78, -89, -74, -71,
    -77, -59, -66, -68, -43, -48, -52, -51, -43, -26, -33, -22, -23, -32, -25, -16, -23, -10, -5, -12, 9, 16, 14, 28,
    44, 40, 36, 43, 43, 42, 33, 45, 48, 60, 38, 28, 39, 42, 41, 41, 31, 22, 21, 13, 28, 16, 14, 9,
    20, 10, 19, 11, 2, 15, 11, 24, 13, 20, 20, 30, 30, 31, 26, 17, 31, 17, 22, 16, 19, 13, 15, 18,
    16, 8, -8, -5, 2, 3, 2, 7, -1, -5, 4, 4, -5, 1, 1, 1, 0, 14, 4, 17, 16, -4, 2, 9,
    19, 6, 4, 3, -6, 9, -7, -7, 5, -8, -10, -15, 11, 5, -7, 0, 5, 5, 6, 11, 2, 13, 14, 1,
    -8, 1, 7, -1, -5, -5, -10, 13, -8, -10, -6, -4, -11, 2, 3, 5, 8, 16, 6, 14, 20, 0, 16, 19,
    17, 26, 33, 22, 13, 27, 22, 31, 25, 31, 33, 23, 26, 28, 29, 29, 31, 38, 40, 40, 26, 37, 47, 54,
    42, 55, 51, 46, 62, 50, 60, 60, 60, 41, 57, 46, 69, 58, 46, 52, 46, 42, 51, 63, 49, 58, 45, 36,
    48, 52, 47, 36, 39, 48, 47, 35, 33, 44, 29, 28, 53, 53, 30, 41, 43, 47, 41, 45, 48, 40, 23, 19,
    50, 45, 22, 13, 17, 15, 20, 26, 25, 12, 20, 17, 16, 17, 19, 14, 6, 9, 12, 11, 5, 5, 0, 22,
    14, 14, 11, 3, 3, 3, 14, 2, 6, 11, 1, -1, 8, -3, -8, -15, -7, -14, -15, -10, -31, -18, -28, -10,
    -31, -22, -29, -38, -25, -37, -31, -37, -46, -48, -45, -35, -40, -38, -50, -59, -47, -43, -56, -56, -47, -52, -56, -46,
    -48, -40, -62, -43, -52, -44, -48, -37, -53, -51, -36, -44, -32, -44, -47, -48, -49, -53, -45, -43, -57, -61, -63, -54,
    -58, -60, -67, -68, -67, -64, -56, -51, -59, -61, -52, -47, -42, -51, -48, -32, -27, -39, -33, -18, 1, -14, 3, 21,
    17, 20, 22, 33, 28, 39, 26, 29, 40, 39, 28, 20, 31, 23, 32, 27, 34, 32, 43, 37, 26, 30, 48, 35,
    31, 47, 42, 37, 35, 60, 56, 48, 49, 59, 66, 52, 69, 80, 80, 67, 71, 75, 66, 66, 73, 80, 73, 57,
    64, 63, 73, 68, 73, 54, 61, 70, 46, 60, 55, 67, 41, 42, 38, 48, 42, 35, 42, 41, 41, 37, 29, 30,
    43, 48, 36, 32, 30, 38, 28, 22, -3, 4, 10, -9, -10, -9, -5, -16, -20, -27, -1, -14, -32, -35, -31, -22,
    -30, -34, -39, -36, -49, -46, -34, -34, -21, -32, -41, -47, -25, -29, -45, -39, -34, -39, -45, -52, -42, -44, -57, -62,
    -55, -68, -69, -63, -68, -59, -53, -67, -68, -60, -54, -60, -68, -71, -50, -58, -56, -47, -59, -46, -28, -25, -54, -34,
    -43, -32, -41, -52, -61, -51, -59, -82, -74, -66, -71, -79, -79, -90, -75, -71, -81, -78, -78, -58, -71, -66, -67, -63,
    -52, -51, -66, -59, -32, -40, -43, -44, -36, -41, -31, -43, -37, -44, -34, -31, -35, -35, -45, -36, -52, -40, -37, -28,
    -38, -39, -37, -42, -31, -29, -15, -14, -17, -14, -22, -13, -12, -5, -10, -11, -21, -17, -10, -17, -9, 0, -3, -10,
    3, -4, -1, 11, 12, -4, 17, 20, 17, 15, 25, 25, 26, 22, 24, 23, 27, 20, 35, 22, 28, 25, 32, 22,
    21, 30, 20, 23, 21, 30, 34, 27, 28, 38, 42, 38, 44, 56, 61, 51, 52, 62, 65, 66, 60, 63, 76, 74,
    64, 74, 70, 67, 61, 66, 78, 82, 77, 69, 82, 90, 84, 82, 79, 70, 85, 90, 80, 81, 73, 70, 84, 82,
    67, 75, 85, 71, 74, 82, 79, 73, 63, 65, 73, 73, 66, 59, 60, 62, 70, 65, 63, 61, 53, 61, 68, 61,
    61, 56, 52, 44, 57, 57, 59, 59, 51, 59, 52, 52, 59, 66, 65, 62, 58, 50, 56, 57, 56, 54, 55, 46,
    40, 38, 32, 51, 38, 33, 38, 34, 30, 32, 29, 24, 27, 16, 12, 16, 20, 21, 14, 2, 8, 4, 8, 2,
    3, 15, 4, -4, -10, -11, -5, -16, -10, -13, -18, -21, -14, -22, -30, -24, -29, -32, -30, -29, -25, -30, -29, -47,
    -46, -36, -29, -39, -58, -39, -33, -35, -51, -41, -38, -38, -31, -42, -30, -39, -37, -31, -41, -45, -35, -41, -45, -37,
    -35, -32, -39, -41, -45, -35, -26, -30, -42, -44, -39, -46, -44, -41, -42, -43, -41, -45, -41, -32, -39, -40, -38, -34,
    -32, -29, -35, -29, -25, -32, -30, -26, -29, -28, -30, -26, -27, -28, -23, -35, -37, -37, -27, -33, -38, -42, -37, -31,
    -39, -36, -38, -44, -49, -42, -34, -29, -33, -34, -25, -20, -27, -23, -20, -16, -16, -10, -22, -21, -23, -15, -11, -9,
    -4, 0, -13, -9, 1, -1, -4, -7, -13, -7, -6, -11, -7, -9, -11, -15, -13, -10, -8, -3, -6, -5, -4, -5,
    -9, -9, -4, -7, -6, -21, -10, -6, -5, -8, -10, -5, 1, 2, -3, 3, -2, 8, 6, 4, 1, -6, -9, -8,
    -3, -3, -8, -6, -11, -15, -9, -10, -18, -10, -19, -19, -14, -12, -15, -14, -21, -25, -19, -15, -15, -23, -21, -17,
    -10, -11, -19, -13, -16, -15, -4, -7, -14, -13, -19, -19, -16, -22, -15, -15, -14, -15, -23, -19, -22, -27, -28, -14,
    -19, -23, -29, -30, -22, -27, -20, -25, -23, -27, -25, -13, -10, -12, -17, -25, -17, -11, -21, -16, -20, -16, -17, -19,
    -24, -19, -18, -18, -15, -17, -15, -18, -12, -17, -18, -12, -5, -6, -13, -4, 0, 10, -1, 7, 2, 7, 15, 13,
    7, 10, 22, 10, 17, 11, 11, 18, 21, 11, 17, 16, 18, 25, 21, 18, 15, 24, 23, 39, 36, 28, 29, 30,
    36, 42, 50, 39, 41, 45, 47, 50, 54, 54, 49, 54, 45, 57, 59, 57, 43, 51, 54, 62, 68, 64, 60, 64,
    62, 64, 69, 73, 71, 67, 69, 75, 76, 74, 75, 76, 76, 67, 66, 76, 72, 69, 63, 66, 62, 65, 56, 60,
    67, 63, 56, 61, 62, 58, 57, 62, 65, 60, 53, 61, 59, 49, 53, 61, 58, 45, 48, 51, 52, 44, 33, 46,
    45, 39, 28, 34, 28, 28, 43, 31, 19, 25, 24, 14, 21, 18, 16, 20, 8, 0, 14, 11, -1, -8, -9, -10,
    -9, -10, -20, -18, -20, -15, -29, -26, -20, -29, -40, -36, -28, -28, -31, -42, -42, -40, -42, -42, -37, -50, -50, -47,
    -49, -53, -46, -53, -51, -47, -49, -50, -48, -53, -63, -55, -46, -51, -58, -54, -54, -64, -60, -57, -49, -53, -46, -48,
    -54, -43, -38, -41, -38, -31, -39, -41, -43, -42, -37, -40, -45, -40, -39, -47, -41, -41, -45, -45, -42, -45, -47, -37,
    -52, -50, -40, -47, -45, -43, -45, -41, -33, -39, -38, -33, -35, -40, -34, -27, -23, -23, -28, -17, -21, -28, -30, -21,
    -22, -27, -21, -17, -11, -15, -15, -9, -12, -8, -6, 2, -10, -11, 1, -5, -10, -10, 4, -1, -6, -4, 5, 3,
    1, 4, 6, 5, 3, 6, 3, -3, 8, 10, 5, -4, 3, -2, 5, 5, 15, 13, 6, 12, 7, 4, 10, 11,
    -3, 2, 5, 1, 9, 1, -5, 0, 3, -3, -2, 6, 4, 10, 5, 5, 9, 5, -6, 9, 15, 4, 11, 10,
    11, 5, 5, 13, 18, 7, 4, 15, 14, 10, 0, 4, 4, 4, 1, 4, 7, -6, -4, -4, 7, -1, 7, -5,
    -6, -3, 4, -6, -2, -6, -7, -5, -19, -16, -5, -12, -19, -23, -15, -15, -15, -30, -20, -19, -19, -15, -24, -30,
    -28, -22, -27, -25, -23, -19, -24, -29, -21, -14, -20, -21, -18, -16, -26, -17, -18, -18, -20, -16, -18, -24, -25, -26,
    -13, -18, -26, -18, -9, -19, -23, -14, -2, -2, -11, -6, -7, -5, -2, 5, 4, 4, 3, 0, 8, 9, 9, 7,
    15, 10, 4, 13, 17, 7, 6, 14, 5, 7, 5, 9, 17, 10, 11, 16, 14, 4, 15, 7, 8, 12, 13, 10,
    12, 15, 11, 18, 19, 22, 24, 24, 20, 16, 24, 28, 31, 24, 26, 22, 26, 29, 38, 34, 33, 32, 34, 37,
    34, 41, 40, 29, 29, 34, 35, 35, 32, 27, 31, 30, 31, 34, 33, 32, 31, 22, 25, 36, 29, 29, 24, 21,
    26, 27, 22, 23, 25, 27, 22, 32, 27, 27, 25, 32, 25, 28, 28, 28, 29, 21, 24, 27, 31, 25, 19, 19,
    23, 21, 13, 22, 27, 24, 11, 13, 19, 16, 11, 15, 18, 7, 15, 13, 10, 6, 14, 16, 14, 13, 7, 17,
    13, 12, 11, 9, 8, 7, 0, -8, -2, 5, -3, -12, -12, -11, -2, -2, -17, -15, -4, -5, -12, -9, -13, -11,
    -11, -14, -13, -11, -7, -12, -18, -13, -15, -4, -13, -15, -19, -7, -16, -21, -13, -15, -20, -13, -19, -26, -22, -23,
    -31, -13, -13, -26, -27, -23, -33, -29, -26, -25, -23, -25, -27, -26, -19, -26, -32, -28, -15, -25, -27, -20, -30, -29,
    -26, -33, -29, -19, -32, -40, -29, -31, -31, -30, -34, -29, -24, -32, -33, -23, -26, -22, -29, -39, -33, -25, -36, -41,
    -35, -38, -38, -31, -30, -29, -32, -39, -46, -35, -29, -23, -35, -38, -33, -37, -32, -32, -37, -31, -23, -27, -28, -27,
    -36, -27, -25, -20, -15, -18, -32, -36, -33, -29, -27, -31, -36, -22, -18, -27, -24, -14, -25, -24, -19, -16, -20, -14,
    -16, -14, -18, -23, -16, -8, -12, -9, -9, -10, -12, -11, -14, -7, -6, -13, -8, 0, -2, -8, -10, -9, -12, 0,
    -1, -4, 1, 5, -6, -6, 9, 8, 7, 0, 15, 15, 9, 11, 20, 18, 7, 15, 15, 17, 14, 22, 15, 9,
    17, 15, 18, 22, 26, 19, 29, 24, 17, 25, 23, 25, 33, 29, 31, 41, 31, 27, 34, 29, 38, 42, 32, 27,
    44, 40, 32, 38, 44, 40, 32, 39, 46, 42, 35, 37, 36, 33, 40, 37, 39, 34, 35, 30, 34, 38, 35, 41,
    36, 34, 36, 35, 32, 37, 39, 29, 37, 37, 31, 37, 50, 48, 41, 41, 54, 52, 44, 45, 56, 46, 40, 36,
    53, 56, 48, 44, 47, 51, 49, 41, 41, 44, 38, 38, 45, 40, 32, 33, 41, 37, 43, 42, 36, 30, 38, 44,
    39, 40, 31, 34, 42, 42, 33, 37, 38, 27, 27, 29, 32, 38, 32, 13, 19, 27, 17, 14, 17, 18, 14, 14,
    13, 13, 15, 13, 13, 11, 11, 11, 10, 8, 11, 9, 7, 7, 3, -5, 2, 12, 6, -4, -2, -9, -13, -4,
    6, 0, -1, -6, -5, -10, -2, -7, -5, -7, -11, -21, -17, -13, -11, -22, -26, -22, -18, -31, -31, -28, -30, -39,
    -40, -38, -36, -43, -43, -38, -37, -41, -38, -38, -43, -43, -41, -46, -46, -44, -32, -40, -45, -51, -45, -49, -55, -47,
    -50, -54, -56, -49, -55, -58, -52, -50, -55, -56, -43, -52, -51, -52, -49, -48, -46, -58, -57, -45, -49, -51, -43, -50,
    -59, -54, -45, -42, -43, -46, -41, -46, -48, -45, -40, -47, -44, -39, -32, -32, -37, -35, -23, -26, -25, -17, -18, -21,
    -15, -6, -1, 4, 4, -1, -2, 4, 2, 2, 6, 5, 1, 3, -3, -11, -10, -3, -7, -12, -13, -11, -16, -14,
    -8, -9, -13, -17, -16, -4, -7, -13, -20, -15, -7, -4, -11, -7, 0, -1, -4, -7, 5, 4, -5, -6, 1, 2,
    -3, -1, -7, -1, 8, 8, 1, 0, 7, 1, 3, 5, 5, 5, 7, -2, 1, 10, 8, -3, 2, 6, 0, 0,
    3, 4, 6, 2, 0, 6, 8, -1, 0, 3, 11, 8, -4, 0, 7, 5, -2, -2, 2, 5, 0, -6, 2, 8,
    2, 3, 8, 13, 9, 3, 8, 15, 11, 5, 5, 9, 8, 7, 7, 18, 11, 5, 13, 17, 14, 11, 13, 29,
    30, 31, 33, 37, 29, 31, 30, 35, 32, 35, 29, 27, 37, 40, 22, 24, 35, 38, 33, 32, 32, 37, 31, 35,
    41, 32, 26, 36, 37, 35, 39, 36, 29, 28, 29, 29, 37, 35, 27, 28, 36, 31, 27, 31, 31, 30, 33, 34,
    35, 38, 38, 43, 42, 36, 43, 46, 36, 36, 43, 39, 39, 36, 32, 39, 34, 29, 36, 39, 20, 32, 34, 29,
    25, 30, 26, 22, 29, 25, 29, 27, 19, 18, 19, 19, 18, 25, 7, 8, 21, 25, 16, 13, 16, 11, 15, 19,
    12, 16, 23, 21, 19, 25, 21, 13, 12, 16, 24, 22, 11, 14, 21, 12, 6, 11, 12, 2, 7, 7, -2, -2,
    3, 1, 2, -2, -3, 2, -6, -13, -8, -4, -6, -9, -17, -15, -14, -21, -25, -15, -13, -22, -23, -18, -18, -23,
    -23, -22, -23, -28, -27, -23, -27, -34, -27, -28, -39, -34, -35, -41, -31, -33, -45, -41, -32, -41, -48, -46, -36, -41,
    -48, -52, -46, -38, -54, -59, -46, -40, -46, -43, -44, -48, -37, -37, -43, -40, -38, -43, -45, -42, -42, -44, -44, -38,
    -43, -50, -48, -39, -43, -53, -50, -46, -39, -45, -47, -48, -40, -43, -45, -36, -33, -34, -36, -34, -32, -28, -19, -21,
    -23, -18, -11, -16, -13, -18, -22, -20, -19, -26, -21, -14, -17, -18, -23, -18, -19, -23, -11, -2, -6, -11, -5, -7,
    -6, 2, 3, -1, 4, 8, 6, 5, 3, 1, 6, 7, 9, 15, 16, 4, 9, 14, 2, 10, 18, 11, 8, 16,
    10, 13, 22, 9, 15, 26, 23, 22, 31, 24, 24, 29, 24, 28, 34, 40, 36, 25, 24, 35, 38, 35, 45, 42,
    37, 30, 43, 40, 28, 25, 25, 23, 22, 22, 18, 27, 23, 15, 17, 21, 23, 24, 27, 18, 20, 18, 24, 13,
    10, 22, 28, 20, 13, 18, 21, 17, 15, 18, 26, 15, 14, 16, 14, 13, 12, 9, 8, 9, 13, 18, 12, 2,
    5, 10, 9, 10, 7, -6, -2, 4, 6, 5, 4, -3, -6, -2, 0, 4, -2, 0, 3, -4, -2, 12, 2, -5,
    8, 9, -1, 1, 1, 4, 2, -3, -7, -5, -11, -9, -9, -7, -7, -7, -11, -10, -13, -12, -16, -17, -2, 0,
    -14, -7, -9, -15, -16, -5, -8, -9, -4, -6, -2, 2, -6, -11, -2, -3, -2, 1, -5, -9, -2, -4, -10, 0,
    5, -3, -6, -1, 2, -2, 1, -7, 9, 18, 6, -7, 7, 18, 8, 10, 14, 12, 6, 8, 12, 9, 8, 15,
    11, 13, 8, 2, 4, 11, 1, 1, 11, 9, 9, 17, 15, 15, 18, 13, 14, 19, 9, 11, 25, 20, 18, 20,
    11, 6, 11, 14, 21, 16, 9, 9, 19, 8, 7, 22, 25, 14, 15, 24, 18, 13, 10, 10, 19, 19, 15, 13,
    16, 9, 8, 9, 13, 11, 1, 3, 12, 10, 2, 5, 3, 7, 7, 6, 6, 13, 5, -3, -1, 7, 13, 7,
    4, 8, 9, 11, 12, 7, 2, 1, 10, 11, -1, 2, 7, -4, -3, 7, 0, -4, -4, -16, -14, -6, -16, -16,
    -18, -21, -13, -16, -27, -28, -19, -12, -13, -18, -17, -11, -12, -10, -11, -12, -12, -18, -19, -16, -16, -17, -17, -18,
    -21, -13, -9, -9, -17, -18, -12, -11, -18, -12, -3, -11, -25, -16, -8, -18, -16, -12, -14, -22, -15, -9, -3, -8,
    -15, -7, 1, -2, -1, -2, -7, -13, -12, -13, -9, -17, -23, -15, -12, -22, -16, -10, -24, -30, -21, -16, -12, -15,
    -20, -15, -18, -25, -25, -11, -3, -8, -10, -4, -4, -12, -10, 2, 1, -3, -4, -6, -1, 0, -15, -7, -2, -13,
    -9, 5, -11, -10, 4, -1, -10, 4, 8, 6, 0, -5, -2, 13, 7, -6, -6, -3, -3, 5, 10, -3, -2, -2,
    -4, -4, 4, -1, -2, -7, -10, -2, 1, -7, -17, -7, -9, -9, 2, -4, -16, -17, -7, -14, -7, -10, -27, -17,
    -14, -21, -18, -11, -24, -18, -8, -19, -17, 1, -8, -15, -9, -4, -11, -8, -2, -12, -17, -14, -9, -10, -14, -9,
    -11, -10, -11, -9, -13, -11, -9, -10, -13, -14, -11, -11, -14, -17, -5, -8, -10, -8, -3, 0, -2, 2, -1, 4,
    8, 5, -3, -3, 8, 2, 10, 5, -1, 5, 6, -1, 3, 7, -1, 2, -2, -2, 6, 2, -2, -1, -4, -1,
    4, -6, -14, -8, -9, -4, -4, 6, 5, -1, -2, 15, 13, 10, 20, 20, 10, 8, 10, 15, 22, 11, 11, 18,
    10, 13, 30, 23, 9, 7, 16, 18, 13, 7, 10, 11, 9, 0, 7, 13, 11, -2, 5, 17, 13, 10, 10, 12,
    9, 16, 20, 11, 11, 14, 8, 9, 17, 11, 2, 9, 12, 12, 13, 10, 11, 8, 5, 6, 12, 16, 13, 16,
    6, 1, 12, 18, 9, 5, 13, 15, 11, 13, 15, 16, 11, 10, 7, 13, 17, 8, 6, 12, 7, 9, 11, 9,
    6, 10, 12, 8, 14, 14, 12, 0, -3, 4, 4, -7, -10, -3, -9, -17, -12, -4, -10, -15, -20, -13, -1, -8,
    -13, -11, -9, -14, -12, -3, -1, -1, -8, -6, -4, 1, 3, 4, 5, 5, 4, 6, 5, 1, 9, 8, 2, -3,
    -11, -3, 7, -4, -13, -2, -5, -9, -6, -11, -9, 0, -8, -9, -2, -2, -5, -10, -16, -6, 1, 1, -2, 0,
    -2, 1, 3, 6, 7, 2, 1, -1, -6, 1, -1, -5, -7, -10, -12, -4, -6, -13, 1, -5, -9, 1, -2, -14,
    -10, -6, -10, -5, 1, -10, -15, -5, -4, 0, 0, -4, -9, -6, -3, -4, -6, -6, -8, -9, -13, -7, -4, -5,
    -8, -12, -15, -13, -11, -6, 0, 3, -6, -10, -3, 9, -1, -3, 4, 7, 1, 7, 7, 4, 0, -1, 6, 8,
    9, 8, 4, -1, 4, 9, 8, 4, 0, -1, 7, 17, 8, 7, 4, 1, 9, 12, 15, 14, 15, 9, 11, 11,
    7, 10, 5, 6, 11, 10, 2, 9, 9, 0, 7, 14, 12, 10, 14, 8, 15, 13, 9, 17, 13, 12, 10, 14,
    12, 15, 9, 3, 11, 4, 4, 7, 9, 5, 1, -3, -2, 3, -1, -5, 5, 0, -10, 1, 14, 7, 6, 8,
    8, 6, 9, 10, 9, 7, 10, 9, 7, 0, 0, 9, 4, 2, 3, 5, 1, 4, 2, -1, -1, -1, 2, 6,
    3, -4, -1, 4, 1, 0, 10, 10, -5, -1, -4, -4, -1, 2, -18, -14, -7, -11, -15, -12, -19, -16, -13, -15,
    -11, -11, -18, -10, -5, -3, 0, 1, -5, -8, 2, 3, 0, -4, 2, -3, -2, 2, 11, 6, -7, -1, 5, 14,
    17, 10, 4, 7, 7, -2, 6, 9, 2, -4, 5, 4, 4, 4, 5, 6, 14, 22, 23, 22, 18, 21, 25, 17,
    10, 19, 15, 5, 12, 10, 8, 9, -3, -8, 5, 11, 1, 5, 7, 4, 4, 2, -2, 5, 11, 9, 12, 5,
    -2, 8, 10, 5, 13, 18, 7, 10, 14, 15, 11, 8, 7, 6, 5, 7, 10, 8, 2, -8, -1, 9, 6, 12,
    14, 2, -1, 8, 6, -3, -1, 0, -5, -8, -2, 2, 3, -7, -17, -9, 3, -4, -10, -5, -8, -15, -18, -17,
    -23, -31, -34, -24, -29, -29, -25, -27, -28, -18, -16, -18, -11, -10, -12, -8, -6, -13, -6, -2, -2, -4, -1, -3,
    -3, -3, -2, -2, -5, -12, -14, -13, -10, -9, -16, -17, -11, -10, -8, -9, -15, -10, 5, 6, 5, 9, 5, -2,
    7, 13, 16, 11, 1, 2, 8, 3, 2, 12, 6, -8, 9, 11, 3, 8, 6, -10, -1, 7, -6, -7, -2, -4,
    -15, -13, -11, -13, -13, -15, -13, -10, -9, -17, -17, -11, -13, -14, -12, -16, -17, -18, -21, -18, -16, -21, -17, -10,
    -16, -15, -10, -9, -14, -13, -11, -11, -12, -15, -15, -20, -31, -24, -15, -23, -29, -23, -26, -28, -31, -36, -21, -15,
    -28, -28, -17, -19, -15, -13, -13, -5, -7, -9, -6, 0, -2, -7, -5, -2, -1, -1, -2, -5, 3, 0, -8, -10,
    -7, -9, -5, -6, -15, -6, -3, -7, -9, -1, 6, 2, -4, 4, 10, 5, 2, 12, 13, 2, 1, 13, 5, -1,
    0, 4, 8, 10, 6, 4, 10, 8, 4, 6, 10, 6, -1, 7, 5, 7, 4, -3, -5, 5, 3, -1, -1, -2,
    3, 2, -9, -7, 3, 0, -3, 2, -4, -1, 1, 0, -4, -3, 1, 8, 13, 2, 4, 13, 12, 0, 7, 21,
    17, 9, 7, 9, 11, 8, 7, 5, 3, 4, 12, 5, 2, 1, 2, 7, 14, 10, 8, 11, 11, 11, 12, 12,
    11, 20, 17, 14, 20, 20, 18, 17, 16, 19, 26, 17, 9, 14, 16, 14, 18, 13, 13, 15, 11, 13, 20, 20,
    15, 18, 21, 18, 16, 30, 27, 19, 23, 22, 24, 29, 27, 22, 24, 25, 28, 21, 18, 24, 27, 27, 28, 27,
    23, 23, 23, 24, 28, 30, 26, 23, 22, 22, 22, 28, 27, 13, 14, 19, 21, 20, 18, 22, 20, 19, 22, 25,
    24, 31, 37, 29, 23, 25, 33, 25, 20, 26, 27, 21, 22, 22, 19, 26, 22, 26, 27, 15, 14, 27, 18, 5,
    16, 23, 17, 8, 5, 12, 12, 7, 10, 18, 12, 11, 9, 11, 13, 16, 17, 11, 13, 17, 19, 22, 18, 8,
    11, 17, 9, 9, 12, 13, 8, 6, 1, 0, 0, 1, 7, 3, 0, 3, 8, 2, -1, 0, 4, 7, 8, 6,
    -2, -7, -7, 3, 3, -8, -6, -6, -16, -14, -7, -12, -14, -15, -22, -18, -8, -12, -21, -24, -25, -22, -14, -19,
    -22, -21, -19, -12, -13, -19, -16, -8, -11, -9, -14, -20, -16, -11, -18, -17, -11, -18, -18, -16, -17, -14, -13, -22,
    -20, -13, -14, -17, -20, -22, -14, -9, -16, -22, -21, -21, -23, -18, -15, -15, -18, -25, -24, -17, -21, -24, -25, -19,
    -20, -25, -22, -20, -24, -19, -20, -23, -15, -11, -23, -22, -11, -10, -19, -18, -10, -8, -9, -13, -8, -8, -15, -18,
    -13, -7, -11, -14, -7, 0, -7, -13, -7, -2, -5, -8, -8, -3, -6, -10, -3, -6, -11, -10, -14, -15, -6, -11,
    -19, -16, -13, -17, -19, -18, -17, -15, -16, -15, -16, -11, -15, -16, -9, -7, -8, -11, -13, -8, -10, -18, -12, -12,
    -19, -11, -11, -21, -19, -20, -29, -23, -22, -21, -20, -31, -30, -14, -19, -23, -14, -14, -15, -13, -11, -7, -6, -14,
    -10, -2, -7, -11, -9, -8, -12, -10, -2, -9, -12, -13, -19, -18, -8, -11, -18, -18, -14, -18, -18, -11, -11, -17,
    -15, -15, -18, -12, -5, -11, -12, -12, -10, -11, -13, -4, -2, -7, -4, -8, -12, -5, 5, -2, -4, 3, 0, -7,
    -1, -1, -4, 3, -2, -4, 5, 3, -7, -1, 2, 7, 5, -1, 6, 16, 6, 3, 12, 12, 4, 5, 8, 16,
    14, 10, 11, 18, 17, 11, 17, 24, 18, 12, 12, 15, 14, 11, 11, 22, 16, 12, 12, 13, 15, 16, 7, 10,
    13, 8, 16, 18, 8, 15, 16, 14, 15, 14, 17, 25, 18, 17, 27, 24, 17, 12, 9, 10, 20, 15, 9, 10,
    13, 7, 7, 12, 9, 3, 8, 12, 15, 16, 16, 13, 10, 8, 19, 18, 14, 20, 23, 12, 9, 15, 15, 12,
    14, 11, 16, 19, 12, 5, 14, 13, 7, 10, 11, 9, 8, 3, 4, 15, 9, -2, 3, 7, 2, -2, 3, -1,
    -5, -11, -7, -5, -3, -8, -8, 2, -1, -9, -3, 4, 1, -2, 4, 2, 3, 0, -2, -5, 2, 1, 0, -1,
    2, 3, -1, -5, -5, -9, -9, -7, -4, -8, -9, -13, -13, -3, -3, -13, -9, -4, -13, -14, -3, -5, -14, -12,
    -7, -6, -5, -10, -12, -9, -10, -9, -4, -9, -15, -15, -15, -4, -3, -12, -12, -8, -13, -10, -5, -5, -8, -5,
    -4, -7, -9, -7, -7, -5, -3, 4, 1, -4, -5, 0, 3, -1, -4, -2, 4, -4, -8, -2, -7, -12, -4, -4,
    -9, -9, -11, -8, -4, -7, -6, 1, -5, -11, -4, 9, 7, -6, -10, -3, -3, -5, -3, 1, -1, -13, -10, 2,
    1, -6, -6, -3, -7, -6, -2, -2, -6, -7, -14, -8, -4, -13, -16, -13, -14, -13, -13, -18, -16, -18, -25, -18,
    -13, -20, -21, -19, -17, -16, -18, -16, -12, -17, -19, -17, -15, -15, -13, -21, -20, -15, -18, -22, -14, -15, -21, -20,
    -20, -18, -17, -25, -23, -19, -18, -20, -22, -24, -27, -22, -20, -23, -25, -23, -21, -22, -20, -12, -13, -18, -16, -13,
    -8, -8, -9, -4, -7, -9, -3, -4, -14, -12, -9, -15, -24, -21, -12, -16, -17, -15, -15, -7, -7, -16, -11, 0,
    -8, -10, -1, 1, -9, -7, -2, -2, -4, 1, -3, 1, 4, -3, -5, 3, 0, -9, -11, -4, -2, -5, -10, -9,
    -4, 0, -7, -9, 6, 10, 2, 1, 11, 12, 9, 13, 15, 14, 7, 4, 10, 19, 14, 7, 9, 12, 8, 14,
    22, 21, 17, 17, 19, 23, 27, 25, 17, 20, 21, 22, 20, 17, 22, 20, 15, 15, 19, 26, 23, 18, 14, 21,
    19, 11, 15, 20, 16, 12, 13, 16, 10, 16, 20, 19, 19, 27, 25, 23, 33, 29, 18, 24, 35, 31, 28, 30,
    26, 23, 27, 27, 20, 17, 22, 23, 18, 22, 21, 18, 16, 8, 13, 23, 17, 11, 14, 11, 13, 16, 14, 15,
    21, 20, 18, 21, 24, 19, 15, 21, 16, 10, 18, 17, 10, 10, 6, -3, 0, 2, -3, -1, -3, -7, -7, -5,
    -9, -11, -10, -14, -11, -8, -11, -17, -15, -14, -12, -10, -9, -6, -8, -16, -11, -9, -8, -11, -13, -15, -16, -12,
    -3, -9, -14, -15, -15, -11, -7, -6, -15, -13, -9, -13, -15, -16, -17, -13, -9, -9, -10, -9, -8, -12, -13, -8,
    -2, -2, -12, -9, -2, -10, -22, -11, -4, -11, -12, -4, -4, -8, -11, -10, -4, 1, -7, -7, 3, 3, -9, -3,
    1, -2, -1, -4, 0, 5, 6, 4, -2, 0, 5, 2, 4, 5, 2, 0, 11, 12, 5, 3, 2, 8, 10, 9,
    8, 0, 5, 9, 6, 9, 12, 4, -2, 2, 6, 9, 7, 4, 2, 7, 9, 10, 7, 7, 12, 13, 12, 15,
    16, 14, 10, 11, 15, 14, 9, 7, 16, 19, 11, 12, 16, 8, 5, 10, 10, 11, 10, -1, -3, 8, 8, 1,
    4, 3, 7, 3, 4, 7, 6, 3, 1, 6, 11, 11, 4, 7, 14, 15, 15, 10, 11, 9, 9, 8, 11, 11,
    8, 6, 5, 7, 6, 5, 4, 8, 5, 8, 13, 10, 6, 9, 5, -4, 4, 11, 5, -3, -1, 5, 4, 0,
    -1, 0, 2, 4, -1, -2, 1, 2, 5, 6, 3, 12, 14, 0, 3, 13, 7, 9, 13, 6, 4, 7, 6, 4,
    11, 7, 5, 7, 3, -3, 7, 10, 5, 5, 1, 5, 15, 4, 0, 11, 13, 1, 5, 8, 0, -2, 3, 5,
    2, 4, 0, -9, 0, 7, -1, 3, 3, -5, 5, 9, 0, 6, 11, 0, 4, 16, 11, 11, 7, 0, 8, 11,
    -2, -5, 5, -3, -2, 2, 0, 3, 4, -5, -2, 9, 6, 6, 6, 5, 11, 13, 6, 7, 10, 3, 7, 10,
    9, 6, 3, 3, 4, 5, 4, 6, 0, 0, 8, 2, 3, 10, 9, -1, 7, 9, 8, 5, 6, 7, 6, 7,
    7, 6, 9, 6, 11, 11, 11, 10, 10, 3, -2, -2, 1, -1, -1, -2, -6, -12, -8, -3, 0, 2, 4, 7,
    11, 9, 6, 9, 14, 9, 14, 21, 18, 17, 21, 14, 12, 18, 17, 13, 9, 7, 10, 5, -1, 4, 4, -2,
    0, -3, -1, 3, -4, -3, 1, -4, -4, -1, 0, 2, -6, -13, -6, -9, -12, -5, -5, -11, -12, -9, -8, -10,
    -9, -10, -10, -6, -5, -4, -6, -6, -9, -12, -14, -9, -11, -15, -20, -22, -21, -12, -12, -21, -23, -24, -25, -24,
    -17, -13, -22, -31, -21, -17, -22, -22, -17, -17, -22, -23, -17, -13, -19, -25, -28, -28, -18, -20, -32, -31, -27, -25,
    -26, -28, -30, -33, -38, -29, -18, -26, -35, -37, -36, -23, -22, -31, -23, -22, -35, -29, -26, -25, -25, -30, -29, -19,
    -20, -23, -20, -21, -24, -21, -25, -21, -16, -17, -19, -15, -16, -11, -13, -7, -4, -5, -3, 2, 3, 7, 7, 2,
    2, 5, 2, 4, 8, 2, 1, 8, 7, 8, 7, 5, 5, 12, 13, 14, 14, 13, 14, 20, 17, 26, 30, 24,
    20, 25, 23, 29, 34, 26, 23, 28, 27, 26, 22, 22, 25, 24, 20, 26, 26, 23, 18, 19, 25, 25, 16, 19,
    27, 20, 19, 24, 23, 22, 27, 25, 19, 23, 21, 16, 20, 24, 24, 20, 20, 14, 17, 22, 25, 19, 20, 25,
    28, 21, 17, 18, 16, 14, 21, 21, 12, 12, 17, 15, 17, 16, 13, 17, 17, 19, 17, 19, 24, 23, 17, 20,
    27, 27, 21, 18, 24, 26, 25, 23, 22, 31, 29, 21, 20, 32, 19, 20, 29, 22, 18, 22, 19, 19, 23, 20,
    20, 23, 15, 14, 21, 23, 20, 19, 17, 13, 18, 15, 8, 15, 22, 12, 7, 13, 14, 11, 12, 13, 12, 9,
    8, 13, 11, 8, 10, 8, 4, 12, 12, 6, 12, 15, 11, 10, 7, 4, 3, 4, 8, 11, 9, 4, 3, 1,
    0, -5, -3, -1, -1, 1, -1, -3, -4, -2, -6, -3, 3, -4, -4, -1, -5, -4, -3, -11, -10, -6, -9, -6,
    -6, -10, -8, -4, -9, -11, -7, -6, -5, -10, -10, -10, -11, -12, -12, -12, -12, -17, -17, -16, -21, -21, -19, -21,
    -24, -24, -25, -31, -29, -27, -31, -36, -27, -30, -37, -36, -32, -27, -20, -29, -33, -25, -27, -31, -24, -22, -23, -26,
    -29, -27, -27, -25, -24, -24, -26, -25, -22, -21, -24, -24, -20, -22, -22, -22, -28, -32, -23, -23, -29, -30, -30, -33,
    -30, -33, -33, -27, -33, -43, -38, -33, -38, -38, -37, -40, -43, -44, -45, -40, -39, -46, -44, -39, -39, -44, -41, -39,
    -40, -40, -41, -43, -43, -43, -43, -42, -45, -46, -40, -44, -45, -39, -40, -46, -44, -39, -46, -45, -38, -39, -41, -39,
    -39, -34, -32, -35, -34, -25, -26, -28, -23, -24, -24, -22, -22, -22, -14, -20, -23, -13, -12, -18, -12, -11, -22, -16,
    -9, -8, -4, -6, -7, -5, -4, -5, -4, -1, -1, -4, -2, 4, 8, 12, 8, 6, 13, 8, 6, 16, 18, 10,
    17, 21, 20, 21, 15, 16, 28, 27, 23, 29, 33, 31, 28, 30, 38, 43, 35, 33, 41, 38, 37, 43, 43, 39,
    36, 45, 49, 52, 52, 45, 45, 48, 49, 52, 56, 53, 52, 54, 50, 48, 54, 58, 55, 54, 57, 60, 59, 55,
    52, 50, 53, 52, 46, 47, 49, 46, 48, 44, 41, 47, 43, 45, 52, 49, 48, 49, 47, 44, 48, 47, 42, 43,
    42, 36, 38, 37, 31, 40, 37, 28, 35, 36, 25, 29, 33, 28, 29, 25, 20, 30, 28, 20, 25, 25, 19, 27,
    26, 22, 22, 23, 18, 25, 27, 23, 20, 17, 13, 15, 17, 12, 11, 16, 10, 7, 13, 13, 6, 5, 5, 5,
    10, 5, -5, -6, -2, -3, -8, -6, -1, -4, -12, -10, -14, -16, -13, -12, -14, -12, -19, -23, -17, -20, -28, -24,
    -25, -26, -22, -24, -25, -26, -31, -32, -29, -35, -34, -27, -34, -34, -26, -26, -36, -38, -34, -32, -33, -35, -31, -33,
    -38, -35, -34, -36, -39, -36, -37, -36, -36, -37, -34, -36, -40, -39, -33, -33, -34, -32, -30, -28, -34, -35, -30, -29,
    -32, -29, -30, -30, -26, -27, -27, -20, -22, -23, -22, -21, -19, -13, -16, -21, -19, -19, -25, -27, -26, -23, -23, -25,
    -26, -25, -25, -27, -30, -25, -22, -21, -23, -21, -22, -25, -26, -22, -24, -25, -20, -23, -24, -23, -25, -22, -21, -27,
    -28, -22, -19, -14, -16, -24, -22, -18, -20, -16, -12, -19, -20, -16, -14, -11, -10, -16, -19, -18, -18, -15, -17, -18,
    -20, -23, -22, -21, -21, -16, -17, -21, -18, -18, -16, -14, -18, -17, -14, -14, -9, -7, -11, -12, -9, -7, -6, -8,
    -8, -7, -7, -3, 0, -1, 1, -1, -4, -4, 0, 4, 0, -4, 1, 4, 7, 3, -5, -1, -1, -5, 1, 8,
    0, -1, 1, -1, 4, 1, 2, 4, 0, -1, 7, 7, 2, 4, 5, 2, 7, 8, 7, 8, 8, 8, 12, 11,
    9, 10, 11, 11, 13, 16, 17, 19, 16, 17, 19, 17, 19, 19, 15, 23, 25, 18, 16, 23, 24, 20, 19, 21,
    20, 19, 22, 26, 24, 19, 15, 19, 25, 23, 19, 20, 16, 13, 18, 25, 24, 18, 17, 14, 20, 27, 19, 15,
    18, 17, 21, 24, 16, 15, 20, 17, 14, 17, 17, 17, 13, 11, 21, 19, 12, 12, 14, 14, 19, 22, 21, 19,
    18, 18, 18, 21, 23, 17, 17, 20, 17, 17, 18, 15, 13, 9, 9, 16, 19, 17, 15, 11, 9, 15, 17, 14,
    14, 17, 14, 12, 14, 13, 10, 13, 15, 11, 16, 19, 12, 11, 11, 11, 13, 8, 8, 13, 11, 10, 13, 10,
    7, 12, 15, 14, 14, 12, 10, 14, 10, 6, 11, 14, 10, 7, 8, 6, 6, 6, 2, 3, 4, 6, 6, 1,
    2, 5, 4, -1, -2, 1, 2, -1, -6, -4, -3, -2, 0, 0, 0, 2, 1, 1, 1, -3, -4, 0, 3, -1,
    -6, -9, -6, -4, -4, -2, -2, -2, -5, -11, -7, -1, -8, -12, -12, -12, -14, -10, -12, -17, -17, -16, -16, -16,
    -16, -17, -18, -16, -14, -12, -14, -16, -15, -15, -15, -15, -14, -14, -20, -18, -13, -15, -17, -16, -20, -20, -15, -17,
    -21, -18, -19, -20, -20, -23, -20, -18, -24, -25, -22, -24, -19, -16, -19, -19, -17, -14, -14, -9, -9, -8, -10, -12,
    -10, -9, -7, -6, -7, -8, -6, -4, -4, -6, -10, -7, -5, -8, -10, -12
};
static const int8_t snd_weak[SND_WEAK_LEN] = {
    16, 10, 23, 5, 10, 23, 24, 39, -5, 7, -15, 13, 5, 27, -8, 15, 3, 5, 7, 2, 11, -2, 12, -6,
    3, 17, 7, 8, 0, -8, 2, -3, 2, 2, 11, 0, 12, -1, -4, 2, -10, -3, -17, -13, -16, -17, -25, -16,
    -14, 6, 9, 23, 24, 41, 44, 41, 49, 48, 51, 45, 43, 27, 30, 29, 53, 45, 56, 65, 45, 67, 50, 51,
    38, 38, 38, 17, 36, 9, 28, 28, 27, 47, 39, 50, 42, 33, 34, 27, 37, 25, 31, 20, 35, 12, 25, 25,
    5, 29, 20, 44, 43, 40, 48, 27, 31, 35, 37, 32, 35, 8, 33, 13, 24, 49, 32, 42, 51, 33, 36, 23,
    15, 11, 10, -11, -1, -17, -20, -9, -33, -16, -29, -20, 2, -3, -15, -28, 2, -9, 1, -25, -33, -23, -38, -31,
    -47, -24, -34, -14, -26, -43, -12, -40, -17, -43, -37, -43, -55, -41, -38, -31, -28, -28, -33, -12, -31, -22, -7, -28,
    -30, -34, -40, -31, -46, -58, -58, -50, -31, -42, -31, -40, -31, -59, -61, -65, -61, -54, -82, -80, -87, -77, -63, -59,
    -51, -48, -32, -45, -51, -47, -58, -42, -78, -59, -71, -58, -66, -59, -57, -58, -26, -43, -15, -37, -40, -41, -50, -46,
    -59, -50, -35, -32, -40, -40, -20, -45, -21, -42, -48, -42, -49, -59, -60, -60, -41, -41, -59, -51, -40, -46, -23, -59,
    -37, -49, -33, -37, -40, -46, -45, -40, -53, -30, -27, -21, -9, -14, -9, -12, 1, 8, 8, -6, -5, -22, -2, -7,
    -6, 7, 6, 5, 18, 29, 30, 14, 32, 22, 42, 24, 26, 36, 39, 41, 22, 42, 43, 56, 57, 58, 56, 51,
    73, 63, 68, 59, 39, 46, 34, 44, 49, 47, 57, 58, 37, 49, 71, 86, 87, 63, 71, 86, 78, 82, 85, 108,
    93, 82, 76, 82, 94, 88, 74, 81, 84, 85, 92, 78, 86, 90, 88, 74, 77, 82, 85, 55, 70, 73, 84, 72,
    54, 73, 66, 81, 62, 45, 60, 67, 80, 55, 63, 49, 69, 46, 48, 41, 38, 72, 51, 52, 44, 38, 37, 61,
    53, 57, 55, 29, 49, 16, 29, 46, 39, 63, 39, 28, 24, 16, 11, 29, 30, 18, 2, -15, 0, 15, 30, -13,
    19, 8, 17, 11, -17, -10, -19, -11, -30, -26, -30, -6, -9, -15, -25, -22, -6, 10, -26, -31, -46, -40, -39, -28,
    -41, -35, -18, -42, -30, -44, -31, -35, -36, -64, -62, -50, -71, -47, -56, -65, -46, -55, -65, -60, -62, -53, -70, -64,
    -72, -39, -72, -55, -69, -61, -66, -59, -56, -63, -53, -62, -45, -43, -38, -43, -40, -41, -54, -57, -78, -49, -58, -65,
    -52, -62, -39, -35, -59, -68, -61, -51, -48, -43, -57, -51, -49, -44, -44, -34, -44, -26, -39, -60, -35, -41, -29, -13,
    -18, -1, 4, -22, -22, -21, -28, -1, -26, -19, -15, -17, -22, -4, 3, -2, 17, -21, -11, -12, 5, 29, 6, 8,
    -1, 4, -1, -2, 5, -12, 3, -23, -4, -15, 4, 22, 24, 31, 11, 19, 7, 16, 8, 15, 12, 3, 7, -12,
    8, -5, 2, 4, 0, 5, 8, -2, -1, -1, 10, -2, 11, -8, -5, -3, -4, 3, -2, 14, 8, 20, -19, -3,
    8, -1, 7, 3, 8, 33, 33, 7, 18, 19, 34, 21, -1, 25, 10, 7, 11, 10, 16, 18, 1, 5, 12, -1,
    9, -12, -19, 0, -32, -9, -17, -4, -2, -26, -17, -20, -1, 7, -7, -11, -20, -17, -6, 3, 1, 4, -3, 15,
    9, 0, 6, 2, 1, -1, -22, -8, -1, -1, -3, -8, -11, -2, 2, -4, 0, -9, -10, -4, -17, -11, 7, -5,
    -16, -25, -15, -1, -1, 2, -4, -3, -4, -13, -10, 3, 2, 5, -3, -23, -10, -8, 6, -5, -7, -6, 5, 9,
    0, 10, 1, 6, -16, 2, 9, 10, 4, -8, -5, 15, 19, -6, 2, -11, 19, 1, 8, 3, 1, 7, 15, -3,
    4, 8, -11, 20, 12, 20, 23, 10, -6, 7, 2, -7, 23, 10, 9, 24, -3, 1, 6, 19, 24, 10, 2, 7,
    22, -9, 6, 2, 15, 19, 19, 16, 18, 18, 20, 28, 17, 17, 28, 31, 21, 21, 14, 28, 28, 29, 31, 33,
    31, 22, 11, 33, 25, 34, 12, 8, 25, 16, 40, 30, 36, 25, 28, 21, 18, 30, 24, 36, 26, 17, 31, 31,
    48, 42, 41, 21, 26, 34, 32, 46, 17, 28, 23, 22, 28, 10, 34, 9, 19, 20, 29, 29, 28, 28, 35, 35,
    31, 33, 26, 23, 23, 26, 25, 33, 24, 24, 29, 17, 20, 28, 10, 3, 13, 5, 24, 10, 24, 27, 27, 29,
    31, 36, 25, 39, 35, 28, 26, 27, 18, 23, 10, 15, 19, 18, 24, 10, 10, 7, 6, 13, 10, 7, 4, 5,
    6, 6, 10, -1, -9, 0, 4, 4, 2, 7, -10, 2, -18, -8, -6, -12, -13, -17, -30, -13, -15, -18, 5, -27,
    -21, -27, -13, -8, -20, -28, -31, -17, -11, -11, -14, -13, -7, -25, -23, -18, -15, -15, -34, -18, -15, -19, -24, -31,
    -19, -10, -31, -20, -23, -17, -13, -9, -21, -13, -36, -33, -13, -30, -27, -19, -40, -27, -43, -50, -42, -34, -37, -30,
    -63, -64, -45, -51, -38, -49, -61, -60, -58, -41, -66, -48, -61, -46, -44, -50, -61, -49, -51, -38, -33, -51, -30, -42,
    -45, -32, -64, -46, -44, -44, -59, -33, -57, -43, -52, -49, -46, -56, -51, -45, -38, -56, -54, -46, -49, -10, -44, -31,
    -39, -26, -21, -28, -5, -23, -3, -13, -11, -21, -44, -14, -11, 3, -22, -15, -32, -7, -14, -15, -1, 11, 6, -8,
    -15, -10, 2, 3, 1, 12, -11, -3, -4, 17, 11, 7, 6, -3, 20, 11, 18, 10, 8, 11, 23, 21, 24, 30,
    3, 31, 12, 32, 15, 38, 43, 56, 38, 25, 35, 28, 51, 34, 27, 21, 37, 31, 25, 26, 30, 44, 28, 18,
    20, 18, 40, 17, 26, 25, 34, 32, 27, 22, 49, 45, 20, 37, 24, 48, 36, 31, 34, 34, 51, 43, 46, 22,
    51, 49, 43, 31, 24, 34, 33, 34, 19, 18, 20, 10, 8, 3, 17, 21, 4, 18, 11, 25, 28, 25, 15, 7,
    14, -11, 10, -7, 6, -11, -10, -10, 2, 5, -1, 1, 7, 18, 13, 1, 5, 11, 9, 7, 2, 9, 6, 5,
    8, 0, 14, 6, 25, 25, 10, 15, 1, 12, -5, 3, 3, -2, -2, -5, -7, -2, 12, 2, 5, 8, 14, 11,
    14, 5, 24, 9, 21, 11, 13, 16, 24, 16, 16, 16, -1, 19, 5, 22, 6, 8, 18, 11, 16, -6, -4, 2,
    -5, 15, 1, 9, 5, 14, 3, 26, 7, 7, 24, 4, 23, 10, -6, 9, 3, 3, 9, 6, 4, 4, -2, 15,
    4, 2, -10, 8, -9, -2, -23, -27, -6, -5, -12, -33, -17, -19, -4, -6, -11, 7, 1, 3, -5, 4, -1, -2,
    0, -10, -20, -27, -4, -12, -8, -10, -25, -5, -4, -2, 4, -4, -13, -21, -15, -1, 3, -8, 7, -9, 7, 5,
    -4, 1, 13, 26, 17, 5, -10, 23, 26, 32, 46, 11, 22, 13, -5, 1, 2, 13, -3, 5, -13, 0, -11, 8,
    1, -5, 9, -22, -9, -14, 10, 1, -2, -10, -21, 1, -14, 2, -14, -15, -9, -10, -6, -3, -6, -16, -13, -22,
    -16, 5, -13, -22, -35, -22, -28, -21, -30, -25, -17, -26, -29, -41, -19, -37, -30, -63, -54, -44, -37, -62, -78, -59,
    -61, -49, -49, -30, -10, -17, -4, -12, 4, -5, -22, -32, -10, 1, 5, 12, -1, 24, 40, 15, 16, 10, 3, -4,
    -11, -21, 0, -7, 3, -13, -13, 8, 0, 10, 10, 10, 25, 15, 2, 6, 27, 34, 36, 30, 4, -2, -4, -14,
    10, -1, 10, -2, -11, -10, 8, 19, 5, 10, -10, -12, -21, -11, 2, -14, 11, -6, 4, 5, 20, 10, -8, 10,
    5, 2, 3, -13, -7, -16, -18, -19, -11, -6, -11, -25, -28, -30, -26, -32, -16, -16, -23, -3, -17, -26, -33, -18,
    -21, -30, -15, -47, -31, -43, -27, -8, 8, -16, -18, -19, -22, 4, -10, -5, -6, -11, -17, -20, -29, -30, 3, 14,
    8, 3, 12, 15, 13, 5, 8, 28, 16, 7, 0, -9, 4, -5, -10, -3, 20, -8, -28, -12, -6, 3, -10, -18,
    -29, -12, -3, -16, -21, -7, 10, 13, -22, -18, -23, -10, -4, 4, 6, 15, 18, 0, 11, 28, 24, 26, -2, -13,
    5, 1, 16, 3, -18, 7, -3, -13, -3, 27, 11, 25, -8, -12, 4, -1, -14, -5, -25, -35, -21, -34, -20, -5,
    -16, 0, 2, -5, 6, -3, -5, 7, 8, 6, -6, -14, -10, 4, -1, 26, 1, 13, 11, 8, 3, 4, -7, 10,
    -10, -4, -7, -15, 4, -13, -7, -4, -1, -10, 9, 6, -11, -4, -6, -3, -7, -5, 5, -3, -7, 2, 11, 10,
    9, -5, -17, -13, -2, 1, 0, 21, -1, 0, -5, 6, 4, 11, -1, -12, -8, -10, 3, 3, 8, 22, 11, 9,
    15, 18, 5, 5, 1, 19, 18, 7, -2, 4, 10, -16, -11, -9, -10, -6, -13, -9, -1, 9, 1, 10, 10, 2,
    -5, -13, -12, 8, 17, 22, 13, 25, 30, 26, 7, 19, 31, 36, 37, 15, 38, 49, 47, 26, 25, 35, 41, 40,
    31, 22, 19, 43, 34, 45, 39, 36, 25, 26, 23, 46, 35, 17, 19, 22, 27, 15, 21, 28, 37, 40, 46, 39,
    43, 41, 46, 33, 44, 27, 41, 12, 26, 23, 9, 29, 34, 39, 27, 41, 19, 18, 28, 11, 15, 14, 9, 1,
    7, 8, -3, 11, 0, 12, 19, 0, 0, -10, 6, 6, 8, -23, -15, -4, -16, -9, -11, 3, 11, 0, -6, -15,
    4, -17, -23, -24, -8, -22, -30, -11, -17, 1, -23, -27, -16, -27, -16, -44, -34, -30, -34, -25, -25, -17, -37, -24,
    -32, -37, -27, -40, -31, -35, -22, -38, -32, -34, -37, -28, -40, -31, -45, -39, -32, -30, -22, -38, -23, -30, -24, -25,
    -27, -26, -28, -21, -37, -39, -35, -35, -44, -43, -37, -40, -45, -49, -57, -36, -31, -35, -29, -43, -17, -27, -25, -17,
    -27, -25, -38, -34, -39, -32, -25, -36, -19, -27, -29, -24, -44, -39, -55, -44, -64, -43, -55, -48, -35, -54, -23, -37,
    -28, -31, -36, -34, -47, -29, -37, -33, -52, -39, -27, -35, -25, -35, -17, -30, -31, -41, -37, -18, -29, -20, -32, -19,
    -13, -11, 3, -4, -7, -13, -7, -3, -5, -4, -1, -1, -9, 12, 2, 20, 12, 5, 9, 30, 22, 25, 24, 28,
    29, 45, 67, 58, 61, 53, 45, 76, 70, 75, 71, 68, 89, 80, 67, 71, 91, 87, 90, 76, 81, 73, 72, 68,
    65, 75, 68, 69, 50, 67, 73, 82, 60, 84, 75, 79, 65, 76, 71, 70, 66, 58, 55, 79, 73, 74, 52, 59,
    56, 56, 42, 41, 54, 30, 41, 37, 40, 59, 41, 38, 39, 47, 42, 33, 38, 41, 34, 32, 10, 17, 25, 35,
    15, 35, 26, 39, 48, 24, 41, 34, 30, 14, 17, -5, 5, 4, -15, 14, 11, 1, 19, 10, 16, 2, 16, 5,
    0, -6, -10, 1, -6, -9, -16, 1, 2, -10, -18, -22, 7, -11, -10, -20, -16, 4, 2, 8, -20, 12, 1, 5,
    3, -2, 29, 14, 17, 10, 10, 16, 30, 4, 10, 38, -2, 29, 6, 3, 30, 7, 0, 1, -6, 3, 4, -18,
    -1, -1, -21, -9, -17, 19, 10, 4, 5, 10, 15, 12, -10, 5, 21, 7, -5, -4, 4, 9, 14, 7, 13, 28,
    21, 12, -4, 9, 18, 1, 17, -1, -8, -13, -26, -37, -28, -20, -10, -3, -29, -8, -3, -7, -11, -7, -14, -13,
    -22, -40, -19, -25, -12, -13, -17, -11, -12, -8, -24, -11, -8, -19, -27, -31, -42, -31, -36, -50, -49, -46, -23, -41,
    -36, -33, -39, -27, -36, -26, -34, -37, -37, -51, -49, -35, -40, -48, -39, -41, -37, -46, -39, -45, -36, -47, -52, -61,
    -71, -43, -55, -57, -47, -50, -29, -37, -33, -48, -35, -41, -31, -52, -36, -22, -38, -39, -47, -52, -32, -49, -47, -63,
    -31, -27, -37, -48, -4, -23, -17, -22, -59, -19, -6, -28, -22, -32, -30, -14, -4, -27, -6, -42, -2, -17, -34, -23,
    -26, -34, -25, -21, -34, -20, -20, -32, -12, -21, -26, -19, -27, -17, -21, -17, -31, -10, 6, 1, -24, -29, -2, 8,
    5, 6, -12, -23, -2, -28, -7, -16, 2, -16, -26, -10, -7, -1, -10, 14, -5, -2, -20, -14, -15, 4, -11, -17,
    -7, -16, -5, -3, -3, 11, 16, 9, 6, 5, -7, 26, -18, 17, 21, 10, 14, 27, 25, 23, 36, 20, 48, 32,
    23, 19, 10, 44, 28, 26, 32, 16, 34, 36, 39, 27, 26, 15, 25, 13, 22, 32, 19, 33, 3, 15, 21, 33,
    14, 28, 20, 35, 32, 7, 19, 12, 11, 5, 9, 6, 24, 19, 20, 14, 25, 35, 40, 49, 29, 35, 14, 33,
    27, 24, 31, 30, 14, 23, 26, 18, 26, 20, 22, 25, 14, 27, 10, 29, 25, 19, 14, 11, 22, 9, 24, 16,
    21, 22, 28, 30, 21, 26, 24, 28, 28, 11, 6, -3, -4, 8, 22, 10, 6, 20, 36, 35, 21, 29, 33, 37,
    36, 14, 19, 22, 38, 37, 32, 35, 33, 19, 21, 25, 27, 26, 15, -2, 13, 30, 15, -4, 12, 2, 23, 12,
    -2, 14, 22, 8, 16, 3, 41, 28, 29, 27, 23, 18, 36, 46, 48, 40, 32, 26, 17, 31, 35, 41, 40, 15,
    31, 21, 29, 51, 48, 28, 14, 14, 19, 9, 12, -2, 10, 0, 12, 8, 19, 14, 19, 19, 30, 24, 15, 20,
    14, 27, 9, 9, 20, 24, 17, -4, -1, -4, 13, -9, 10, 13, 11, -1, 0, -21, -9, -14, -23, -26, -25, -30,
    -14, -32, -23, -7, -12, -18, -20, -6, -17, -14, -26, -23, -14, -19, -13, -38, -16, -32, -29, -26, -28, -25, -18, -41,
    -35, -34, -24, -16, -30, -34, -29, -35, -39, -46, -45, -31, -19, -35, -38, -36, -39, -29, -38, -35, -22, -39, -35, -54,
    -58, -45, -28, -38, -44, -36, -32, -33, -35, -40, -36, -42, -45, -33, -48, -26, -30, -44, -31, -35, -41, -41, -41, -56,
    -46, -32, -45, -42, -38, -43, -30, -37, -38, -36, -34, -17, -21, -38, -35, -27, -25, -26, -22, -25, -15, -13, -21, -37,
    -16, -7, -19, -23, -21, -17, -20, -18, -19, -21, -6, -9, -13, 0, 6, -1, 5, 10, 5, 11, 12, 8, 0, 7,
    -3, 4, -11, -3, -11, -12, -5, -24, 6, -3, -2, 8, -12, -8, -5, -19, 0, -15, -9, -1, 8, 3, 6, 1,
    19, 23, 11, 19, 9, 9, 11, 2, 7, 11, 22, 24, 6, 14, 18, -3, 11, 19, 24, 19, 29, 17, 34, 22,
    39, 23, 44, 45, 28, 27, 25, 33, 42, 30, 50, 33, 34, 30, 45, 22, 41, 23, 33, 38, 26, 13, 19, 29,
    30, 24, 20, 20, 26, 15, 25, 21, 41, 20, 21, 13, 18, 33, 18, -1, 16, 29, 24, 23, 6, 21, 47, 30,
    42, 21, 24, 31, 16, 9, 16, 20, 35, 14, 20, 22, 8, 21, -2, -12, 3, -3, 3, 9, 17, 11, -5, -17,
    3, -3, 14, 36, 17, 6, 10, 10, 30, 34, 37, 27, -1, 0, -8, -2, 14, 0, -26, -27, -18, -22, -33, -22,
    -14, -6, -13, -12, -14, -20, -11, -17, -18, -28, -32, -25, -33, -24, -37, -28, -26, -18, -16, -18, -7, -17, -28, -25,
    -19, -6, -28, -35, -35, -31, -32, -34, -39, -19, -23, -14, -35, -11, -14, -8, -28, -30, -26, -33, -33, -33, -28, -39,
    -29, -33, -15, -11, -27, -16, -23, 0, -15, -15, -24, -20, 1, 1, -2, -36, -16, -19, -20, -11, -17, -18, -19, -16,
    -30, -11, -19, -25, 0, -19, -12, -6, -8, -1, -4, -17, 3, -11, -17, -19, -19, -3, -5, -7, 14, 12, 12, 11,
    3, 3, 23, 7, 12, 2, 2, 4, -7, -9, 0, -18, -12, -35, -9, -6, -9, -12, -13, -31, -21, -18, -30, 0,
    3, -19, -7, -6, 10, 2, 17, 2, 20, 8, 21, 23, 18, 27, 22, 15, 26, 12, 14, 34, 34, 43, 34, 12,
    44, 36, 23, 6, 16, 15, 17, 11, 14, 4, 26, 14, 4, 15, 8, 10, -4, 7, 6, 9, 14, 1, 9, 16,
    25, 28, 30, 35, 47, 51, 34, 46, 47, 32, 28, 24, 34, 18, 31, 9, 9, 18, 17, 12, 21, 12, 26, 24,
    9, 34, 17, 10, 5, -1, 9, 23, 16, 15, 0, 3, -7, -1, 2, 1, 1, -5, 0, -16, -8, 0, 1, -6,
    -9, -12, -7, -12, -21, -13, -10, -10, 4, -28, -11, -20, -16, -27, -24, -25, -32, -33, -39, -43, -30, -27, -23, -24,
    -26, -32, -16, -38, -22, -26, -34, -31, -41, -37, -28, -43, -49, -50, -46, -24, -22, -38, -33, -36, -34, -38, -42, -43,
    -24, -41, -41, -44, -35, -41, -32, -49, -24, -28, -32, -27, -32, -24, -17, -36, -20, -23, -23, -22, -23, -9, -3, -2,
    12, 8, 10, 14, -2, 17, 16, 10, 12, 2, 7, -6, -3, 9, -6, 8, 8, -9, 3, 13, 14, 9, 11, 13,
    8, 11, 11, 2, 13, 17, 8, 9, 20, 14, 32, 28, 28, 21, 11, 30, 18, 30, 15, 32, 35, 29, 25, 38,
    48, 54, 43, 34, 35, 71, 57, 53, 56, 41, 45, 51, 35, 47, 58, 41, 58, 37, 61, 40, 43, 55, 21, 23,
    35, 26, 32, 32, 27, 22, 39, 20, 46, 38, 24, 24, 13, 33, 21, 25, 10, 22, 25, 18, 29, 28, 33, 24,
    28, 14, 18, 9, 20, -5, 10, 10, -2, 21, 15, 8, -8, 10, 12, 26, 31, 11, 8, 9, 17, 12, 13, 23,
    10, 4, -1, 12, 6, 2, 20, -5, 3, -3, 1, 9, 19, 5, -8, -28, -21, -14, -11, -9, -8, -8, -1, -8,
    -2, -11, -1, -33, -14, -8, -19, -13, -25, -16, -15, -10, -16, -12, -18, -8, -9, -35, -14, -28, -7, -20, -24, -8,
    -20, 6, -11, -20, -7, 2, -11, -22, -17, -20, -15, -29, -28, -36, -21, -34, -29, -23, -29, -8, -46, -38, -27, -16,
    -23, -22, -34, -30, -31, -25, -24, -7, 8, -2, -6, 7, 2, 29, 10, 16, 15, -6, -7, -21, -2, -5, 8, -9,
    -20, -7, -21, -11, -16, -9, -13, -13, -14, -12, -18, -14, -10, -6, -14, -24, -17, -17, -13, -10, -33, -15, -17, -8,
    -16, -26, -19, -10, -8, -27, -19, -24, -17, -20, -20, -14, -27, -23, -32, -21, -21, -12, -33, -16, -20, -8, -11, -12,
    -9, -22, -14, -28, -26, -11, -21, -24, -37, -23, -24, -14, -20, -17, -7, -9, -2, -12, -23, -11, -12, -13, -10, -19,
    -14, -12, -19, -13, -9, -7, 5, -12, -3, 4, -8, 14, -1, -7, 3, -14, 6, -4, 0, -7, 0, 5, 5, 13,
    2, 11, 18, 23, 11, 6, 27, 28, 11, 12, 23, 14, 20, 10, 9, 21, 20, 9, 19, 21, 20, 10, 13, 3,
    24, 8, 2, 5, 18, 23, 11, 15, 27, 34, 34, 5, 21, 31, 27, 12, 14, 26, 20, 15, 16, 29, 20, 33,
    15, 22, 24, 8, 11, 3, 3, 5, 14, 15, 11, 4, 7, 10, 13, 13, 12, 12, 3, -2, -1, 3, 9, 11,
    11, 2, 16, 33, 25, 30, 21, 30, 32, 29, 18, 17, 28, 37, 36, 22, 29, 37, 47, 51, 38, 44, 37, 36,
    41, 38, 51, 49, 48, 37, 22, 23, 21, 44, 37, 46, 39, 40, 35, 45, 44, 29, 16, 31, 34, 41, 57, 54,
    44, 72, 58, 55, 49, 42, 38, 63, 50, 39, 29, 39, 25, 36, 30, 24, 31, 21, 8, 22, 18, 32, 32, 5,
    -7, -2, 1, -6, -2, 2, -2, -9, -25, -24, 19, 15, 4, 11, -1, 3, 2, -7, -3, 11, -7, -28, -34, -14,
    -5, -2, -10, -31, -29, -18, -20, -27, -28, -32, -38, -42, -51, -55, -61, -48, -45, -40, -39, -46, -35, -48, -37, -49,
    -45, -48, -57, -23, -44, -45, -42, -31, -37, -32, -34, -25, -30, -32, -40, -40, -22, -26, -23, -25, -24, -10, -19, -14,
    -30, -48, -7, -14, -22, -29, -37, -41, -30, -33, -28, -22, -29, 5, -25, -25, -16, -20, -15, -18, -22, -18, -23, -2,
    -5, -2, 2, -5, -24, 4, -14, -9, -14, -28, -33, -33, -11, -40, -45, -36, -20, -18, -27, -16, -14, 17, 11, -4,
    14, 16, 13, 5, 1, -10, -7, -7, -14, -10, 3, 21, 18, 14, 7, 15, 8, 9, 25, -15, -2, -12, -36, -10,
    0, 2, -6, -17, -18, -12, -1, -8, 11, -1, -4, -17, -23, -17, -14, 2, -7, -17, 8, -3, 1, 17, 13, 22,
    53, 26, 22, 23, 25, 0, 5, -3, 13, 22, -14, 8, 17, -7, 5, 17, -10, -16, -26, -33, -18, -22, -38, -27,
    -6, 11, 6, 0, 7, 2, 10, 33, -9, -17, 3, 1, 3, 4, -32, -11, 8, -7, 4, -4, -2, 0, 10, 12,
    -11, -2, -18, -14, 3, -12, -21, -5, 0, 5, -4, -16, -15, 8, -3, -20, -27, -13, -13, 3, -18, -21, -7, -11,
    -18, -19, 8, 17, 24, 9, -12, 1, -18, -32, -40, -1, -7, -33, -33, -42, -21, -24, -33, -13, -35, -36, -37, -36,
    -29, -37, -37, -35, -29, -29, -43, -3, -25, -41, -26, -28, -25, -24, -31, -36, -29, -38, -40, -36, -41, -32, -30, -38,
    -18, -20, -29, -24, -25, -21, -18, -19, -5, -20, -4, 5, -7, -6, -14, -2, -1, -6, -8, -12, -9, -13, -8, -5,
    3, -7, -10, -9, -7, -7, -10, -13, -8, -2, 2, -6, 0, 7, -8, 8, 5, -7, -3, -3, -30, -5, 5, 18,
    20, 7, -3, 17, 18, 12, 11, 9, 9, 16, 11, 4, 13, 18, 12, 24, 27, 33, 30, 17, 5, 27, 33, 9,
    9, 15, 17, 10, 21, 26, 17, 3, -3, 14, 17, 11, 3, 0, 17, 23, 12, 13, 22, 19, 10, 24, 33, 24,
    28, 23, 6, 17, 34, 8, 5, -3, -5, -22, 8, 0, 1, 6, 9, 0, 1, 0, -14, -33, -12, -12, -10, 5,
    -7, -15, 1, -5, -12, -4, -1, 0, -11, -16, -20, -4, -14, -18, -29, -17, -18, -6, -6, -20, -17, -10, 7, -2,
    0, 4, 5, -4, -6, 4, 15, 24, 17, 30, 26, 26, 16, 14, 20, 18, 22, 27, 7, 10, 2, 13, 30, 29,
    13, 18, 40, 43, 50, 44, 34, 30, 53, 60, 62, 60, 23, 40, 46, 41, 60, 32, 10, 21, 22, 22, 38, 34,
    68, 48, 46, 50, 29, 46, 49, 22, 24, 16, 27, 25, 39, 41, 35, 47, 45, 53, 44, 61, 60, 60, 42, 26,
    24, 44, 40, 47, 39, 42, 49, 39, 42, 43, 17, 38, 40, 36, 48, 13, 10, 0, 10, 1, 12, 32, 9, -7,
    5, 11, 6, 10, 5, 3, -5, -19, -32, -17, -3, 1, -41, -35, -19, -13, -17, -8, -12, -11, -8, -21, -12, -10,
    -13, -29, -12, 5, -7, 6, -9, -9, -6, -9, -15, -18, -14, -24, -20, -18, -49, -40, -47, -48, -55, -37, -45, -42,
    -36, -36, -33, -36, -44, -28, -29, -44, -37, -32, -35, -36, -29, -12, -30, -34, -36, -40, -35, -40, -45, -49, -51, -43,
    -56, -42, -38, -50, -46, -49, -50, -47, -56, -33, -38, -34, -40, -34, -31, -39, -43, -42, -43, -40, -47, -41, -50, -47,
    -40, -39, -40, -47, -37, -51, -46, -47, -49, -51, -47, -43, -44, -33, -27, -34, -25, -30, -20, -31, -30, -12, -14, -17,
    -12, -40, -30, -20, -25, -5, -17, -26, -28, -20, -10, -9, -9, 0, 11, -1, -26, -12, 2, 12, 19, 6, 15, 13,
    12, 7, 24, 25, 20, 12, 16, 22, 32, 28, 31, 32, 41, 55, 39, 51, 64, 36, 61, 51, 30, 57, 61, 65,
    61, 66, 72, 78, 80, 77, 62, 84, 84, 73, 65, 65, 65, 83, 83, 85, 76, 73, 75, 73, 61, 63, 75, 62,
    71, 75, 60, 53, 60, 53, 83, 66, 59, 52, 63, 45, 53, 54, 39, 33, 43, 41, 49, 61, 36, 47, 43, 40,
    44, 44, 52, 50, 20, 22, 24, 20, 26, 29, 20, 29, 24, 14, 24, 22, 15, -4, 13, -6, -16, -11, -2, -1,
    6, 4, -11, 7, 21, -1, -15, -13, -15, -9, -2, -1, -6, -15, -26, -27, -20, 8, -11, -10, -12, -24, -35, -30,
    -24, -42, -34, -42, -27, -31, -29, -39, -37, -37, -29, -25, -32, -24, -27, -27, -35, -26, -41, -43, -44, -47, -61, -50,
    -57, -56, -52, -56, -51, -58, -31, -49, -41, -27, -41, -34, -24, -24, -25, -43, -45, -29, -35, -58, -48, -39, -25, -36,
    -48, -45, -19, -12, -33, -33, -39, -57, -37, -35, -19, -29, -25, -25, -41, -21, -17, -7, -11, -19, -28, -12, -23, -22,
    -19, -16, -25, -38, -38, -16, -26, -34, -26, -33, -32, -20, -30, -28, -30, -22, -4, -11, -3, -12, -9, -1, -4, -21,
    -15, -14, -35, -43, -27, -31, -41, -33, -32, -27, -39, -40, -29, -40, -30, -32, -39, -23, -10, -9, -12, 2, -11, -21,
    -1, 7, 3, 1, 1, -4, -1, -11, 4, 3, 14, -9, -14, 4, 6, -1, -2, 3, 5, 0, -9, 0, 20, -10,
    -11, 0, 1, -1, 6, -6, 7, 31, 19, 11, 4, 10, 2, 17, 12, -2, 12, 6, -10, 1, 27, 27, 26, 21,
    36, 25, 20, 13, 15, 32, 20, 13, 13, -6, 26, 4, 19, 22, 23, 26, 17, 26, 9, 19, 32, 24, 31, 9,
    -1, 21, 26, 37, 49, 52, 69, 50, 43, 56, 39, 52, 30, 29, 18, 27, 23, 16, 13, 25, 20, 5, 21, 30,
    25, 28, 21, 14, 28, 20, 13, 16, 29, 12, -1, 13, 16, 8, 26, 24, 12, 35, 13, 14, 13, 15, 10, 13,
    24, 23, 23, 21, -3, 30, 38, 14, 23, 15, 16, -3, -8, -3, 2, 27, 8, 2, 17, -2, 16, 32, 17, 38,
    22, 8, 16, 30, 57, 43, 37, 28, 21, 33, 39, 35, 34, 26, 13, 15, -19, 4, 29, 30, 38, 27, 20, 25,
    23, 18, 15, 23, 35, 39, 50, 37, 36, 38, 49, 53, 47, 30, 22, 14, 27, 20, 3, 8, -17, -10, -23, -18,
    -15, -13, -9, -2, -13, -20, -12, -27, -23, -18, -44, -19, -25, -46, -24, -21, -29, -40, -31, -40, -39, -25, -42, -56,
    -36, -45, -19, -36, -59, -56, -46, -34, -29, -27, -35, -43, -26, -30, -25, -20, -37, -54, -43, -54, -78, -73, -75, -68,
    -53, -55, -74, -78, -61, -45, -34, -39, -35, -12, -26, -37, -46, -23, -41, -45, -36, -26, -20, -38, -38, -18, -5, 3,
    -9, -7, 5, -4, 7, -12, -24, -28, -24, -25, -34, -37, -22, -19, -5, -3, 3, 23, 15, 22, 17, 17, -4, 11,
    2, 6, 25, 20, 25, 25, 9, 8, 18, 15, 20, 14, 11, -6, -5, -14, -3, 6, 10, 7, 0, 2, 0, -15,
    -2, -8, -13, 2, 2, 0, -1, 8, -1, 11, 8, -1, 3, 4, -4, 9, -3, -6, -5, -9, -5, -15, -8, -2,
    -5, -12, -13, -12, -22, -25, -11, -6, -9, -21, -37, -29, -22, -23, -15, -21, -25, -29, -23, -25, -17, -25, -13, -10,
    -4, -2, -4, 3, 9, 12, 15, 6, -6, 6, -6, 4, -4, -10, -13, 6, 1, 6, 1, -6, 7, 12, 7, 6,
    0, -3, 14, 11, 20, 8, 18, 10, 25, 20, 13, 8, 5, 9, 8, -3, 10, 1, 2, 22, 13, 16, 9, 23,
    30, 30, 16, 13, 24, 23, 21, 24, 13, 13, 29, 14, 40, 51, 43, 43, 53, 37, 41, 46, 36, 31, 29, 29,
    19, 5, 28, 37, 35, 30, 19, 26, 34, 32, 20, 30, 33, 33, 24, 22, 20, 25, 43, 39, 41, 24, 26, 25,
    21, 28, 28, 24, 28, 23, 6, 12, 21, 20, 18, 32, 15, 15, 13, 15, 22, 27, -5, 13, 15, 20, 34, 20,
    23, 23, 30, 27, 20, 26, 21, 20, 25, 6, 3, 6, 1, 20, 12, 5, 3, 6, -4, 11, 7, 22, 17, 14,
    2, -2, 4, -2, -7, 5, -3, -18, -9, -20, -18, -33, -23, -10, -7, -23, -31, -15, -18, -19, -17, -19, -19, -17,
    -25, -10, -12, -13, -28, -19, -24, -18, -7, -13, -27, -20, -32, -31, -31, -39, -30, -20, -36, -58, -40, -37, -50, -44,
    -44, -49, -33, -47, -47, -42, -47, -43, -42, -36, -55, -34, -41, -41, -37, -44, -36, -44, -46, -39, -47, -48, -48, -47,
    -48, -46, -39, -35, -45, -50, -46, -37, -42, -41, -38, -32, -30, -47, -44, -51, -47, -47, -48, -45, -53, -59, -45, -40,
    -39, -37, -33, -29, -14, -28, -40, -27, -32, -20, -5, -14, -11, 1, -6, 13, 21, 13, -2, 18, 22, 27, 30, 14,
    19, 30, 31, 25, 41, 52, 68, 51, 46, 39, 61, 57, 56, 60, 51, 53, 49, 46, 54, 56, 60, 48, 54, 50,
    46, 69, 43, 52, 55, 59, 61, 65, 66, 61, 60, 79, 82, 59, 58, 80, 88, 69, 66, 78, 84, 68, 65, 73,
    85, 73, 69, 45, 56, 43, 56, 51, 52, 67, 47, 64, 59, 58, 61, 66, 48, 48, 44, 52, 53, 50, 42, 37,
    30, 30, 22, 26, 29, 30, 30, 23, 33, 16, 26, 25, 24, 21, 15, 14, 3, 2, -3, -10, -3, -14, -10, 3,
    -30, -25, -30, -24, -7, -27, -29, -39, -31, -24, -29, -34, -40, -13, -26, -20, -37, -46, -12, -16, -23, -24, -46, -25,
    -11, -31, -29, -46, -45, -42, -36, -50, -39, -34, -40, -31, -52, -64, -54, -43, -36, -43, -51, -44, -50, -33, -30, -29,
    -27, -22, -25, -40, -42, -53, -29, -4, -31, -45, -42, -44, -25, -47, -50, -37, -33, -37, -32, -51, -64, -50, -52, -48,
    -55, -59, -65, -57, -36, -53, -73, -50, -48, -65, -54, -56, -79, -56, -60, -54, -37, -52, -46, -34, -37, -43, -34, -21,
    -17, -40, -47, -42, -45, -40, -48, -25, -9, -18, -22, -18, -24, -20, -3, -20, -30, -19, -40, -24, -30, -35, -35, -27,
    -27, -51, -40, -31, -19, -14, -14, -5, -2, -7, -15, 3, -8, -23, -5, -11, -2, -8, -14, -1, 13, -4, -10, -10,
    -3, -4, 3, 8, -3, -5, -13, 6, 6, -6, -3, -5, 6, 9, 16, 12, 14, 21, 6, 23, -3, -1, 20, 30,
    35, 17, 9, 26, 22, 16, 12, 19, 16, 14, 13, 1, 16, 12, 9, 22, 24, 36, 33, 12, 31, 40, 35, 19,
    15, 15, 12, 14, 9, 23, 21, 7, 7, 4, 7, 7, 3, 2, 10, 16, 3, 11, 16, 3, 21, 27, 11, 26,
    14, 30, 11, 8, 11, 4, 8, 4, 13, 15, 19, 7, 13, 18, 8, 0, -2, -2, -5, 5, -3, 3, 1, -1,
    5, 2, 6, 17, 30, 35, 25, 24, 14, 8, 15, 5, 14, 22, -5, -11, 2, 14, 5, 22, 22, 15, 5, 2,
    10, 15, 8, 13, 6, 22, 35, 14, 17, 12, 16, 24, 16, 13, 19, 13, 10, 20, 21, 11, 12, 15, 19, 30,
    17, 11, 6, 8, 18, 9, -1, 13, 12, 6, 6, -2, 3, 9, -9, 2, 9, 0, 10, 12, 8, 14, 8, 8,
    7, 20, 13, 21, 13, -4, 5, 11, 8, -11, -3, 2, 0, 3, 4, 4, -5, -1, 0, 4, 4, -17, -8, -5,
    -7, -14, -8, -5, -9, -7, 1, 12, 0, 0, 10, 7, 5, 8, 0, 5, 3, -8, -3, 9, -2, -14, -11, 0,
    -6, -11, -9, 3, 7, 1, -2, -9, 3, 0, -3, 1, -5, -11, -15, -14, -3, -8, -11, -15, -9, -18, -11, -11,
    -11, 6, -12, -5, -4, -6, -7, -23, -26, -22, -14, -7, -15, 5, -2, -9, -1, 2, 1, -4, 10, 6, 0, 8,
    -11, -2, 12, -7, 2, 4, -7, -3, -3, 2, 9, 17, 1, -3, 1, 0, 9, 4, 3, 3, -7, 9, -4, -6,
    1, 8, 5, -3, 2, -5, 5, -2, -3, 8, -7, 0, 7, 4, -2, 1, 13, 17, 7, 2, -4, -3, 5, -5,
    0, 7, 7, 9, 1, -12, -11, -3, -11, -17, -6, -12, -14, -6, -21, -11, -14, -17, -15, -11, -15, -31, -28, -26,
    -19, -17, -23, -18, -14, -18, -5, -14, -12, -3, -10, -15, -24, -21, -23, -12, -7, -7, 2, -11, -21, -11, -2, 8,
    9, 14, 7, -1, 2, -1, -7, 11, 10, 6, -7, -1, 8, 12, 8, 13, 13, 0, 19, 25, 31, 26, 10, 10,
    21, 17, 28, 12, 22, 26, 26, 33, 24, 36, 41, 40, 51, 54, 53, 46, 37, 44, 41, 35, 36, 40, 44, 42,
    44, 39, 36, 34, 13, 26, 29, 19, 13, 10, 28, 29, 25, 13, 24, 21, 12, 25, 35, 42, 37, 17, 17, 32,
    41, 43, 46, 44, 33, 18, 7, 16, 25, 8, 16, 17, 0, 7, 9, 11, 11, 8, 0, -12, 0, 6, 16, 14,
    6, 7, 4, -3, -3, 17, 23, 13, -1, -5, -7, -23, -30, -17, 3, 3, -15, -16, -14, -12, -12, -12, -9, -13,
    -12, -9, -30, -17, -23, -20, -25, -24, -23, -23, -28, -24, -27, -28, -53, -36, -38, -39, -31, -39, -50, -30, -25, -16,
    -14, -20, -30, -27, -38, -33, -12, -11, -5, -11, -10, 2, -2, -23, -19, -12, -8, -21, -28, -30, -36, -32, -27, -27,
    -29, -27, -24, -36, -21, -37, -47, -25, -33, -31, -35, -40, -34, -20, -32, -42, -43, -23, -16, -32, -32, -15, -11, -21,
    -32, -39, -36, -36, -30, -24, -31, -20, -27, -20, -15, -24, -34, -33, -36, -16, -31, -33, -20, 0, -6, -16, -22, -19,
    0, -3, 5, 14, 7, 19, 13, 28, 26, 20, 13, 4, 27, 20, 9, 13, 27, 31, 30, 27, 2, 12, 24, 20,
    30, 26, 31, 9, -17, -4, 8, 3, 31, 28, 39, 47, 26, 50, 78, 68, 53, 61, 50, 35, 43, 36, 39, 45,
    43, 50, 24, 35, 36, 40, 43, 35, 44, 37, 34, 23, 13, 22, 31, 17, 0, 3, 2, 6, 21, 9, -3, 4,
    -8, 7, 15, 13, 22, 10, 21, 28, 17, 18, 12, 25, 16, 14, 0, -12, 7, 9, 5, -1, 1, -4, -9, -25,
    -37, -39, -33, -48, -63, -53, -57, -59, -62, -52, -41, -44, -42, -41, -29, -21, -23, -18, -16, -19, -26, -29, -18, -24,
    -29, -34, -40, -31, -32, -40, -32, -22, -20, -27, -28, -39, -18, -25, -23, -6, -18, -6, -8, -15, -16, -13, 2, 7,
    4, 5, 3, 0, -5, 14, -4, -5, 0, -18, -4, -1, -20, -7, -13, -20, -20, -17, -25, -19, 0, 4, 2, -12,
    -35, -21, -18, -14, -3, 4, -5, -4, 6, -10, -3, 15, 10, 3, 12, 4, 15, 19, 19, 10, 18, 14, 19, 30,
    20, 8, 9, 13, 15, 27, 25, 26, 21, 14, 25, 26, 27, 20, 12, 17, 8, 29, 25, 30, 27, 15, 26, 18,
    35, 33, 31, 28, 27, 35, 39, 32, 19, 19, 27, 27, 30, 39, 32, 27, 18, 22, 35, 34, 28, 34, 18, 14,
    11, 13, 15, 29, 22, 27, 19, 7, 18, 19, 9, 9, 15, 25, 15, 6, 12, 21, 23, 25, 31, 29, 26, 15,
    10, 23, 16, 9, 23, 20, 18, 14, 13, 24, 32, 20, 16, 9, 9, 16, 27, 15, 17, 23, 28, 16, 10, 12,
    20, 27, 23, 12, 15, -3, 4, 15, 13, 8, 14, 8, 19, 16, 11, 14, 23, 19, 12, 10, 4, 5, 12, 17,
    6, 3, 11, 1, 21, 20, 9, 14, 5, 10, 4, 4, -4, 4, 19, 15, 17, 7, -1, 8, 6, 12, 11, 22,
    6, 2, 8, 11, 8, 21, -1, 0, 15, 19, 15, 9, 5, 2, 6, 1, 17, 24, 3, 11, 19, 5, 11, 11,
    7, 9, -6, -14, -10, -4, -14, -9, -19, -26, -30, -19, -13, -13, -18, -16, -12, -11, -19, -16, -9, -14, -13, -18,
    -31, -29, -24, -21, -28, -27, -24, -20, -14, -18, -18, -22, -27, -27, -29, -27, -20, -12, 1, -2, -4, -5, -5, 4,
    -2, 12, 11, -2, 1, 4, 10, -1, -9, -11, -1, -5, -18, -20, -22, -13, -21, -10, -5, -16, -29, -11, -9, -15,
    -14, -11, -2, 2, -9, 9, 3, -9, -6, -4, -6, -17, -18, -7, 3, -10, -21, -17, -16, -25, -7, -17, -23, -14,
    -23, -14, -19, -19, -11, -22, -10, -8, -20, -26, -29, -35, -28, -3, -14, -14, 2, -17, -9, -2, -18, -26, -16, -21,
    -41, -28, -12, -18, -6, 3, 24, 27, 17, 22, 23, 2, -6, -7, -12, -14, -18, -24, -38, -39, -40, -29, -21, -38,
    -33, -44, -44, -49, -44, -48, -53, -44, -38, -53, -59, -40, -28, -28, -36, -37, -24, -33, -22, -27, -49, -38, -26, -36,
    -24, -16, -29, -26, -32, -34, -29, -41, -36, -24, -36, -35, -40, -50, -39, -40, -36, -37, -32, -41, -37, -41, -51, -46,
    -56, -46, -38, -39, -54, -60, -59, -48, -36, -41, -18, -26, -31, -30, -31, -12, -11, -8, -17, -24, -42, -42, -26, -27,
    -21, -20, -27, -29, -40, -30, -13, -1, 0, -9, -10, -8, 7, 4, -12, 4, 2, -1, 5, -4, 2, 10, 25, 32,
    38, 30, 34, 33, 37, 35, 38, 35, 37, 23, 17, 24, 21, 25, 33, 25, 22, 28, 32, 40, 40, 31, 46, 37,
    26, 39, 32, 47, 54, 47, 53, 62, 57, 48, 56, 58, 67, 58, 50, 65, 69, 67, 67, 64, 71, 79, 70, 62,
    61, 49, 48, 58, 51, 57, 43, 33, 43, 38, 41, 38, 41, 33, 24, 22, 22, 16, 9, 13, 6, 5, 8, 7,
    18, 25, 19, 18, 17, 14, 23, 21, 18, 18, 4, 8, 7, 18, 14, 11, -4, -1, 5, 0, 8, -6, -4, -2,
    -3, 2, 2, 2, 1, 8, 0, 0, 3, 1, -4, -6, -8, -11, -4, -12, -13, -4, -3, 0, -9, -10, 3, -5,
    -6, -9, -4, -4, 0, -2, -12, -9, 4, -3, -7, -13, -21, -18, -13, -15, -12, -13, -19, -30, -24, -14, -22, -31,
    -18, -17, -23, -26, -26, -17, -12, -18, -11, -18, -19, -22, -30, -17, -22, -35, -26, -19, -27, -27, -24, -34, -17, -18,
    -24, -25, -30, -21, -7, -11, -13, -14, -5, -11, -16, -22, -18, -16, -15, -19, -21, -12, -20, -18, -15, -24, -19, -21,
    -23, -23, -21, -16, -11, -6, -2, 5, -4, -12, -14, -6, -10, -2, -2, -7, -12, -16, -10, -4, -11, -12, -5, 3,
    -4, 3, -4, 1, -2, 12, 8, -2, -7, 4, 4, 10, 19, 19, 18, 17, 10, 16, 21, 22, 16, 31, 29, 28,
    16, 15, 16, 12, 13, 7, 9, 12, 16, 22, 25, 15, 31, 32, 22, 22, 9, 15, 21, 7, 2, 10, 12, 12,
    19, 29, 29, 28, 23, 21, 28, 31, 11, 7, 8, 5, 5, 10, 1, -6, 17, 14, 10, 15, 14, 24, 24, 10,
    8, 16, 5, 3, 5, 1, 1, 1, -6, -5, 6, -2, -5, 5, 9, 14, -5, 8, 4, 10, 13, 11, 7, 0,
    -5, 1, -4, -5, 0, 10, 10, 4, -6, 2, 8, 3, 2, 5, -9, -8, -9, -25, -10, -21, -24, -27, -16, -21,
    -18, -17, -21, -8, -9, -14, -15, -24, -28, -27, -30, -27, -25, -26, -26, -27, -12, -7, -18, -10, -11, -15, -21, -10,
    -13, -13, -15, -15, 1, -3, -5, 0, -6, 7, 2, 4, 10, 14, 3, -4, 2, 10, 10, 9, 5, 8, 3, 3,
    -2, -9, 0, -1, -1, -2, -16, -3, -1, -3, 15, 20, 10, 7, 17, 16, 32, 17, 7, 19, 11, 5, 3, 12,
    23, 19, 17, 15, 19, 21, 27, 17, 22, 22, 17, 30, 30, 31, 37, 33, 35, 43, 45, 33, 29, 32, 24, 26,
    20, 26, 21, 19, 13, 15, 17, 21, 24, 18, 18, 20, 15, 16, 17, 18, 31, 27, 15, 30, 38, 32, 39, 42,
    46, 30, 41, 31, 28, 27, 31, 24, 26, 14, 23, 16, 7, -4, 9, 1, 4, -5, -3, 14, 6, 1, 13, 3,
    -10, -5, -7, -10, -17, -24, -16, -12, -19, -15, -21, -12, -10, -18, -21, -2, -15, -20, -24, -30, -34, -54, -48, -48,
    -45, -54, -51, -44, -53, -55, -52, -54, -59, -64, -60, -57, -67, -67, -58, -56, -63, -63, -54, -57, -60, -68, -67, -63,
    -76, -60, -70, -66, -68, -72, -61, -69, -72, -82, -72, -66, -79, -83, -84, -88, -90, -94, -88, -85, -84, -78, -68, -53,
    -54, -46, -49, -66, -46, -58, -50, -53, -63, -61, -61, -47, -41, -36, -16, -21, -19, -22, -31, -29, -22, -22, -29, -40,
    -41, -32, -33, -32, -37, -14, 4, -4, -26, -2, 4, -1, 17, 10, -1, -9, -17, 1, 22, 35, 46, 66, 66, 48,
    67, 67, 51, 65, 67, 70, 62, 49, 34, 46, 65, 65, 80, 72, 67, 72, 60, 59, 51, 68, 64, 47, 54, 44,
    44, 55, 43, 34, 32, 35, 34, 36, 43, 27, 21, 7, 6, 11, 19, 18, -2, 7, 5, 6, 13, 3, 21, 19,
    17, 33, 31, 34, 22, 21, 16, 10, -5, -13, 6, -15, -28, -20, -25, -3, -7, -12, -5, -14, -14, -25, -12, -17,
    -8, -12, -16, -21, -19, -16, -14, -15, -13, -5, -13, -25, -28, -26, -12, -6, -27, -28, -26, -28, -31, -33, -39, -24,
    -38, -48, -48, -33, -33, -32, -20, -10, -14, -13, -7, -1, -2, -18, -20, -13, -18, -17, -13, -21, -15, -16, -17, -9,
    -16, -11, -3, -11, -14, -13, -14, -10, -3, -2, 2, 9, -15, -4, 4, 18, 19, 21, 20, 5, -3, 18, 25, 17,
    8, 5, 3, -6, -10, -17, -12, 15, 10, 9, 11, 14, 19, 12, 0, 8, 37, 39, 45, 46, 29, 39, 38, 31,
    30, 27, 29, 23, 15, 10, 9, 11, 15, 17, 36, 15, 36, 44, 36, 41, 41, 43, 46, 39, 45, 46, 50, 49,
    55, 49, 59, 54, 55, 58, 52, 43, 60, 41, 48, 43, 35, 56, 57, 55, 56, 60, 64, 49, 49, 58, 45, 45,
    39, 49, 40, 25, 25, 44, 50, 44, 40, 28, 26, 35, 26, 22, 22, 20, 42, 42, 35, 32, 26, 30, 42, 33,
    42, 43, 34, 30, 19, 20, 20, 33, 24, -3, -17, -35, -42, -38, -53, -41, -33, -38, -43, -33, -29, -6, 0, -13,
    0, -2, 0, 1, -17, -18, -14, -13, -21, -25, -20, -28, -29, -16, -15, -23, -31, -25, -20, -30, -29, -44, -42, -39,
    -45, -33, -34, -41, -33, -23, -30, -42, -26, -16, -13, -19, -30, -31, -40, -38, -44, -31, -30, -44, -43, -37, -21, -14,
    -22, -6, -12, -6, -7, -15, -12, -4, -3, -22, -30, -24, -17, -11, -13, -27, -19, -23, -22, -31, -11, -13, -13, -16,
    -19, 1, 5, 15, 21, 11, 33, 34, 29, 33, 17, 18, 19, 18, 13, 33, 23, 15, 14, 6, 1, 16, 10, 21,
    34, 16, 30, 30, 18, 21, 34, 32, 38, 36, 17, 7, -2, -8, 0, 11, 16, 9, -2, 8, 15, 14, 21, 12,
    24, 28, 16, 18, 3, 4, 36, 31, 29, 26, 22, 22, 14, -3, -7, 9, 15, -1, -11, -21, -22, -19, -5, -7,
    -3, -18, -24, -14, 8, 1, 0, -15, 14, 7, 12, 13, 32, 37, 27, 26, 10, 14, 14, 35, 27, 19, 14, 8,
    -2, 5, -5, 9, 19, 13, 8, 4, -14, -11, 3, 1, 9, 2, -10, -8, -21, 2, 10, -13, 8, -18, -20, -24,
    -18, -2, -11, 3, -2, -3, 6, 10, 17, 16, 18, -3, -13, -9, -21, -5, -6, -14, -6, -26, -37, -50, -33, -25,
    -14, -6, -10, -23, -40, -44, -26, -31, -7, -13, -20, -31, -26, -15, -18, -22, -19, -6, -1, -9, -8, -2, -11, -15,
    -17, -17, -23, -23, -28, -35, -22, -27, -6, -17, -8, 16, 0, -3, -17, -25, 1, 5, -12, -26, -13, -21, -29, -34,
    -47, -17, -6, -14, -18, -40, -25, -10, -4, -15, -27, -20, -31, -37, -44, -24, -16, -5, -3, -2, -12, 0, -4, 1,
    1, -12, -1, -6, -10, -7, -11, -8, -4, -13, -18, -6, -4, -6, 6, -3, -4, -19, -11, 1, 2, 1, -14, -17,
    -9, -15, -6, 1, 0, 4, 4, -7, -18, -15, -17, -27, -5, -22, -17, -14, -20, -13, -18, -18, -14, -6, -5, -1,
    -12, -15, -3, -5, 5, 1, -6, -5, -15, -5, -9, -5, -2, -13, -20, -15, -23, -3, -10, -9, -8, -16, -15, -12,
    -20, -13, -16, -11, -16, -9, -16, -11, -6, -11, -6, -14, -2, -8, -13, -4, -4, -10, -12, -7, 11, 14, 9, -1,
    9, 8, 13, 10, 16, 18, 29, 27, 24, 18, 29, 30, 23, 22, 11, 14, 11, 27, 14, 20, 21, 24, 14, 17,
    11, 22, 17, 16, 9, 12, -10, 10, 1, 2, 3, 0, 5, 11, 11, 13, 7, 18, 15, 21, 23, -1, 12, 18,
    20, 13, 16, 27, 30, 26, 24, 19, 39, 17, 22, 21, 11, 16, 15, 27, 26, 16, 6, 14, 19, 10, 18, 5,
    10, 12, 22, 13, 2, 9, 33, 24, 9, 12, 8, 12, 8, 5, 10, 17, 2, 3, 6, -5, 6, 12, -5, 3,
    -9, 1, -2, 10, 8, 3, 9, 4, 8, 7, 2, -11, -16, -25, -13, -4, -7, -17, -9, 2, -13, -19, -23, -18,
    -4, -16, -7, -18, -24, -10, -5, 4, -13, -1, -2, -11, -12, -13, -19, -12, -23, -25, -12, -21, -23, -24, -24, -37,
    -14, -10, -18, -16, -18, -12, -13, -17, -6, -3, 0, -4, -14, -2, -7, -6, -2, -2, 11, 11, 8, 3, -12, -9,
    10, 1, 12, 21, 11, 4, 2, 3, 33, 21, 31, 25, 20, 20, 22, 15, 39, 29, 21, 23, 22, 14, 23, 28,
    16, 20, 19, 13, 19, 12, 10, 15, 6, -5, -4, 5, 15, 6, 11, 8, 18, 12, 36, 36, 30, 17, 14, 13,
    27, 28, 28, 28, 26, 18, 11, 24, 14, 16, 17, -2, -1, -4, 4, 15, 14, 8, -7, -7, -6, 6, -3, 8,
    12, 0, -1, -6, -12, 2, -2, -11, -1, -8, -20, -19, -24, -19, -2, -21, -19, -18, -18, -23, -23, -15, -6, -6,
    2, -13, -10, -8, 4, 2, 3, -17, -17, -37, -44, -43, -28, -32, -16, -26, -53, -21, -28, -14, -6, -5, -5, -20,
    -26, -14, -11, -11, -19, -12, -34, -26, -18, -20, -21, -15, -16, -25, -26, -31, -15, -18, -27, -31, -28, -34, -22, -28,
    -26, -12, -7, -21, -9, -41, -18, -11, -12, -17, -8, 11, -2, -1, -9, -6, 11, 22, 13, 36, 15, 18, 18, 16,
    16, 24, 30, 33, 31, 21, 16, 32, 21, 16, 12, 14, 36, 12, 24, 22, 29, 17, 8, 20, 23, 46, 61, 56,
    43, 61, 70, 78, 75, 79, 73, 68, 49, 43, 48, 66, 58, 60, 60, 61, 66, 60, 55, 49, 47, 64, 51, 36,
    57, 71, 50, 42, 41, 31, 39, 48, 36, 34, 28, 42, 48, 49, 49, 35, 57, 59, 34, 37, 26, 37, 38, 36,
    36, 24, 29, 17, 8, 21, 21, 36, 13, 11, 22, 5, 11, -3, -1, 14, 3, -7, -18, -23, 2, 6, 1, -10,
    -14, -5, -24, -26, -38, -13, -18, -20, -18, -18, -11, -14, -17, -13, -21, 1, -8, -15, -15, -25, -16, -32, -35, -25,
    -10, -32, -45, -55, -44, -50, -49, -64, -59, -59, -70, -67, -71, -50, -53, -56, -55, -58, -64, -48, -54, -42, -51, -38,
    -37, -50, -39, -38, -34, -30, -28, -35, -47, -51, -41, -51, -27, -48, -46, -54, -61, -73, -72, -70, -57, -51, -44, -32,
    -34, -24, -29, -40, -31, -27, -26, -21, -24, -20, -26, -28, -31, -21, -37, -30, -46, -54, -49, -49, -53, -39, -45, -43,
    -49, -53, -51, -50, -42, -51, -42, -45, -34, -20, -24, -11, -16, -16, -8, -7, -15, -15, -6, -27, -30, -18, -29, -17,
    -30, -22, -13, -17, -11, -10, -2, 2, 16, 3, 1, 8, 13, -9, -2, -21, -8, 7, -9, -20, -1, -1, 4, -4,
    -10, -13, -1, 0, -7, -3, -16, 12, 4, 15, 8, 11, 30, 36, 25, 9, 11, 10, 34, 18, 11, 10, 11, 7,
    1, 7, 31, 4, 19, 14, 25, 4, 30, 24, 21, 36, 22, 10, 3, 11, 34, 26, 5, 25, 10, 37, 14, 11,
    34, 52, 45, 35, 29, 31, 35, 25, 24, 42, 43, 18, 14, 0, 29, 23, 27, 25, 32, 3, 3, 7, 25, 21,
    27, 16, 12, 15, 22, 29, 28, 22, 33, 27, 27, 20, 23, 36, 24, 17, 18, 37, 2, -13, 5, 10, -17, -6,
    -17, 15, 6, -4, -11, -1, 2, 12, 14, 5, 12, 11, 12, -3, -10, -21, 14, 18, 0, 6, -17, -12, 11, -3,
    23, 16, 18, 39, 39, 25, 34, 32, 24, 44, 34, 33, 19, 3, 29, 19, 6, -18, 2, 23, 49, 24, 27, 15,
    6, 11, 3, 6, 32, 20, 17, 14, -4, 36, 35, 28, 22, 38, 50, 61, 17, 12, 30, 32, 20, 26, 3, 27,
    23, 1, 29, 23, 38, 55, 27, 21, 45, 18, 26, 36, 19, 27, 0, -11, 5, 24, 21, 26, 16, 16, -5, 6,
    -12, 2, -7, 3, 20, -11, -12, -7, 5, 3, 19, 14, 9, 26, 29, 9, 26, 16, 12, 26, 26, 17, 11, 10,
    2, -11, -9, -19, 6, 3, -25, 1, -12, 16, -20, -15, -18, 2, 2, -26, -10, -6, -10, -21, -38, -30, -30, -10,
    -22, -17, -19, -9, 1, -28, -17, -31, 3, -18, -29, -22, -27, -16, -42, -40, -21, -31, 10, -35, -31, -16, -43, -35,
    -51, -34, -40, -37, -36, -43, -25, -63, -23, -41, -37, -49, -32, -32, -39, -45, -24, -22, -42, -35, -31, -46, -23, -6,
    4, 37, 16, 3, 13, -6, -2, -3, -3, -18, -25, -33, -49, -23, -34, -31, -18, -35, -14, -57, -27, -40, -28, -30,
    -31, -24, -38, -36, -27, 3, 6, -3, 0, 5, 10, 28, 21, 32, -2, 16, 4, -8, 2, 5, 9, -18, -7, -26,
    -3, 11, 17, 28, 27, 12, 17, -3, -9, -18, 8, 3, 3, -24, -9, 7, 7, -30, -13, 17, 3, 6, -20, 5,
    -8, 10, 15, 4, 6, 2, 21, 0, -9, 10, 13, 37, 12, 13, 14, 9, -4, -6, 7, -51, -27, -56, -47, -38,
    -12, -26, -18, -4, -18, -9, -19, 0, 25, -4, -7, -13, 4, 4, -17, -19, 7, -6, -19, -21, -7, -16, -1, -26,
    16, -9, -9, -6, -13, 6, -1, 5, -15, -16, -18, 19, -12, -6, 0, 15, 20, -15, -14, -2, 10, -2, 10, 15,
    7, -14, -17, -21, 6, -14, -4, -12, -5, 2, 3, 6, -5, 10, 17, 25, 0, 16, -4, -2, -16, 3, -18, -20,
    -28, 1, -35, -22, -24, -19, 4, -10, 30, 10, -12, -4, -2, 6, 7, 18, 22, 20, 18, -8, 28, 16, 45, 46,
    59, 41, 35, 3, 6, 9, -1, -20, -30, -22, -12, -28, -3, 10, -9, 17, 12, -15, -23, -1, -12, 20, -1, 19,
    7, 21, 15, 48, 15, 54, 69, 12, 30, 25, 23, 27, 26, -27, -7, -12, 3, 24, 19, 32, 54, 77, 64, 87,
    71, 99, 64, 52, 28, 25, 10, 39, 16, -4, 15, -17, -5, -17, 5, -4, -3, -2, -6, -10, -14, -22, -13, -10,
    -12, -11, -43, -13, -12, -24, 1, 13, 17, 40, 28, 30, 21, 29, 34, 8, 30, 1, 32, 6, 35, 11, 36, 14,
    22, 20, 5, 2, 11, -26, -18, -28, -28, -22, -35, -28, -30, -16, -27, -32, -1, 6, -2, -2, 8, 10, -9, 25,
    18, -2, 14, -11, -3, -19, 4, -20, -16, -9, -22, -43, -15, -14, -32, -19, -35, -28, -21, -35, -12, -34, -10, -19,
    4, -12, -7, -6, -26, -7, -28, -11, 12, -19, 28, 14, -9, -7, -4, 17, 17, -16, -11, 29, 7, 2, -8, 10,
    24, 21, 7, 18, 5, -1, 34, -19, 31, 10, 10, 3, -17, 0, -8, 6, -27, -13, -7, -16, 19, -24, 16, -10,
    -16, -28, -23, 3, 9, 8, -30, 0, -15, -27, 7, -7, -4, -12, -16, -42, -25, -38, -7, -5, -16, -27, -44, -55,
    -13, -35, -37, -22, -30, -30, 4, -17, 0, 22, 5, 40, -3, -2, 8, 44, 0, 36, 9, 27, 18, 3, -7, 22,
    0, -21, -26, -9, -29, -17, -14, -43, 5, -14, -26, -45, -37, -43, -6, -29, -34, -9, -26, -37, -32, -27, -10, 21,
    12, 20, -22, -29, 10, -5, 20, 59, 44, 53, 49, 25, 57, 44, 76, 49, 64, 69, 69, 49, 40, 74, 52, 59,
    46, 24, 51, 3, 11, 31, 8, 32, 23, 36, 9, 29, 30, 45, 23, 23, 51, 30, 31, 57, 42, 52, 47, 32,
    34, 24, 21, 23, 16, 5, 28, 24, -3, 20, 11, 6, 15, 4, 3, -3, 3, 0, 14, -6, 15, 7, 8, 14,
    1, -10, -33, -5, -25, -19, -13, -11, 5, -24, -31, -33, -23, -36, -34, -22, -45, -28, -44, -46, -38, -47, -38, -45,
    -42, -51, -47, -52, -59, -46, -54, -33, -45, -51, -48, -32, -13, -27, -18, 14, -6, 6, 4, -6, 15, -5, 18, 7,
    -9, -22, -5, 1, -16, 40, 12, 21, 23, 7, -5, -4, -8, 14, -15, -2, -2, -15, -11, 17, -9, 20, 1, 44,
    17, -11, 16, 15, 35, 25, 52, 35, 46, 37, 13, 56, 24, 49, 34, 24, 22, 32, 26, 35, 15, 21, 29, -3,
    9, 10, 23, 7, 30, 0, 14, 14, 17, 18, 23, 33, 26, 8, 23, 6, 16, 11, 8, -1, -1, 8, -15, -5,
    -13, 2, -18, 5, -8, 0, 0, -1, 7, -10, 0, 1, 3, 6, 10, -10, -13, -8, -9, -16, -18, -17, 10, -34,
    -28, -35, -22, -29, -26, -26, -36, -28, -32, -14, -55, -7, -16, -33, -25, -47, -4, -23, -29, -8, -31, -9, -9, -31,
    -51, -14, -50, -4, -26, -19, -12, -21, -26, -1, 11, -14, 22, -13, -21, -29, -29, -23, -28, -19, 2, -28, -27, -38,
    -7, -7, 20, 24, 16, 24, 17, 25, -15, 17, 3, 1, -23, -35, -27, -35, -15, -23, -14, -29, -14, -13, -25, -44,
    -17, -12, -24, -26, -10, -14, -15, -25, -22, -19, 16, -22, -2, 2, -10, -14, -22, -14, -15, -9, -26, 1, -41, -43,
    -15, -39, -19, -17, 8, 14, 2, 2, -25, 11, -21, 12, 4, -13, -13, -15, -7, -11, 0, 12, 3, 18, -31, 2,
    -6, 16, 14, -21, 15, -3, 14, 27, 18, 51, 41, 41, 16, 30, 13, 26, 26, 16, -4, 24, 0, 35, -19, 3,
    11, -8, 3, 16, -3, 11, 1, -16, -14, -7, -15, -4, 3, -15, 1, -10, -10, 18, 39, 48, 65, 32, 43, 40,
    40, 49, 38, 45, 25, 19, 11, 19, 30, 11, 42, 29, 26, 18, 32, 16, 15, 10, 10, 27, 17, 17, 21, 6,
    20, 39, 35, 47, 52, 32, 40, 17, 16, 48, 25, 31, 29, 6, -4, 5, 3, 16, 22, 13, 29, 12, 6, 1,
    19, -1, 9, 0, 4, 19, 11, -15, 9, 0, 1, -6, 15, 12, 11, 13, 1, 4, 5, 4, 36, 10, -10, 19,
    8, 1, 6, -5, 29, 12, 8, -13, -1, -10, -20, -16, -48, 21, -27, -33, -31, -56, -40, -31, -39, -47, -11, -37,
    1, -14, -18, 9, 18, -10, 35, -4, 26, 16, 13, -6, 17, 9, 16, 15, 4, 8, 20, 6, 8, 25, -6, 27,
    17, 0, 8, -1, 13, -19, 13, 2, 13, -1, -14, -1, 0, -5, 12, -20, -7, -29, -4, -6, 10, 5, -2, 30,
    -17, 7, -9, 33, 5, 13, 49, 5, 27, 17, 21, 21, -6, -2, 1, -35, -8, -3, -2, -14, 3, -28, 8, -3,
    -1, -12, -3, -13, 12, -19, -17, -6, -11, -9, -28, -39, -31, -41, -1, -36, -28, 15, -6, -8, -25, -16, -26, -12,
    -6, -26, -16, -16, 14, 2, -7, -27, -16, -11, -53, -53, -24, -55, -34, -46, -39, -38, -59, -58, -44, -59, -55, -49,
    -68, -52, -47, -36, -48, -9, -26, 6, -9, -5, -6, -11, -16, 2, -34, -8, -25, -12, -28, -48, -33, -40, -26, -31,
    -35, -18, -23, -38, -54, -51, -40, -49, -47, -43, -25, -33, -36, -4, -44, -19, -13, -14, -11, -37, 1, -22, -6, -11,
    -21, -13, -15, -15, -26, -33, -6, -39, -7, -32, -32, -23, -48, -19, -41, -29, -18, -39, 0, -22, -7, 4, 10, -10,
    -21, -1, 0, -17, -5, -2, 35, 23, 12, 3, -1, 14, 15, 27, 21, 12, 15, 0, -7, -8, 0, 2, 20, -5,
    -7, -12, 7, -19, 21, -8, 4, -3, -3, -13, 2, -9, 10, -14, -12, -10, -2, -13, -5, -10, -11, -6, -15, -13,
    -20, -33, -10, -21, 7, -32, 16, -20, -9, -27, 8, 14, -1, -5, 1, -8, 21, 10, 20, -2, 2, 18, 20, -3,
    11, 19, 16, 7, 23, 14, 35, 42, 14, 17, 4, -27, 34, -13, 6, -30, -17, -12, -12, -3, -17, 25, -27, 5,
    9, -2, -14, -4, -25, 25, -4, 3, 5, 8, 12, -14, -2, -33, -1, 3, 17, 12, 47, 20, 2, 1, 0, -9,
    32, -1, -18, -3, -12, 15, 2, 27, 14, 25, 26, 42, 24, 29, 49, 47, -2, 23, -8, 4, -27, -5, 20, -33,
    -34, -20, -53, 3, -21, -15, -4, -24, -12, -24, 1, 1, 48, 30, 22, 31, 20, 2, 45, 24, 31, 30, 22, 10,
    6, 0, -15, 29, -7, 39, 17, -1, 31, 25, 16, 15, 26, 15, 49, 33, -3, 39, 7, 51, 60, 30, 48, 55,
    50, 46, 35, 43, 59, 53, 69, 59, 62, 37, 69, 32, 40, 33, 42, 68, 27, 56, 67, 54, 45, 61, 69, 73,
    53, 85, 73, 88, 62, 88, 81, 74, 102, 90, 77, 64, 84, 76, 90, 92, 101, 85, 82, 72, 69, 81, 46, 66,
    34, 48, 33, 45, 27, 60, 23, 22, 8, 18, -7, 14, -20, -2, -13, -4, 19, -25, -7, -11, 1, -16, 28, 3,
    27, 11, 27, 22, 6, 34, 54, 46, 31, 63, 16, 37, 15, -2, -23, 15, -26, 5, -16, -33, -29, -38, -44, -40,
    -43, -59, -28, -34, -56, -51, -57, -66, -42, -36, -61, -43, -50, -61, -27, -63, -52, -26, -45, -12, -23, -33, -45, -28,
    -17, -42, -30, -44, -34, -27, -47, -53, -57, -62, -65, -56, -47, -51, -52, -63, -80, -75, -66, -56, -43, -48, -33, -53,
    -33, -52, -29, -11, -21, -18, -24, -20, -31, -36, -12, -40, -27, -40, -57, -33, -52, -41, -41, -41, -54, -27, -59, -52,
    -72, -68, -52, -71, -47, -60, -53, -73, -65, -44, -57, -38, -50, -58, -57, -85, -52, -43, -47, -46, -34, -39, -61, -40,
    -37, 5, -25, 8, -22, -35, -6, -9, 17, -11, 12, -36, 0, -10, -23, 20, -8, 20, -1, -3, 15, -18, -10, -8,
    21, -11, -7, -3, -25, 4, -4, 0, 1, -2, -1, 6, 4, 10, 32, 4, 61, 38, 30, 44, 37, 59, 20, 43,
    47, 61, 30, 58, 21, 25, 44, 10, 40, 31, 37, 38, 47, 18, 58, 37, 52, 41, 74, 31, 72, 62, 63, 54,
    84, 42, 65, 63, 48, 79, 59, 52, 43, 58, 34, 23, 18, 2, 21, -12, 12, -8, -5, 0, 6, 6, -11, 8,
    -12, -7, 9, -14, -30, -23, -17, -26, -6, -22, -22, -32, -6, -40, 20, -38, 8, -6, -9, 13, -5, -3, -7, 0,
    -2, -4, 14, -8, -8, 0, -13, 8, -24, -23, 10, -3, -16, -40, -10, -12, -2, -31, 2, -19, -13, -1, -2, -34,
    -25, -36, -28, -34, -31, -48, -39, -53, -22, -31, -44, -11, -12, 3, -2, -17, -5, 13, -26, 27, 34, -9, 27, 23,
    32, 14, 21, 24, 25, 5, 26, 13, 0, 1, 5, 19, -9, 9, 3, 8, 10, 26, 15, 29, 15, 40, 22, 27,
    17, 5, 25, 12, 24, 13, 15, 12, 5, 11, 7, 3, 7, 3, -7, 1, -5, -10, -15, -15, -7, -18, -11, -18,
    -8, -35, -10, -12, 29, 25, 35, 10, 15, 25, 22, 43, 11, 18, 30, 4, -16, 5, -25, 9, 1, 3, 4, -10,
    -16, 11, -4, 26, 25, 26, -28, -3, -40, -22, -9, -20, -8, -3, 2, 1, 0, 13, 28, 32, 35, 19, -6, -32,
    -5, -34, 6, -8, -16, -12, -27, -14, -53, 4, -43, 22, -47, -17, -35, -25, -24, -1, -24, 2, -3, -25, 15, -24,
    -7, 5, 5, -2, 19, 3, -17, 12, -26, 16, 19, -6, 11, 7, -21, -4, -13, -36, -5, -16, -21, -6, -33, -23,
    -14, 17, 1, 10, 9, -2, 20, 4, 18, 36, 19, 52, 30, 33, 32, 12, 42, 18, 21, 10, 4, 6, 15, -2,
    32, 19, 24, 20, 27, 0, 21, 12, 23, 19, 12, -17, 7, -20, -5, -9, -23, -15, 6, -22, -11, -2, 6, -12,
    17, 6, -1, 11, -17, -13, -18, -21, -28, -21, -20, -35, -7, -43, -28, -17, -40, -14, -30, -51, -90, -46, -39, -50,
    -15, -14, -12, -8, -14, 22, 39, 50, 79, 101, 49, 68, 57, 51, 73, 35, 38, 26, 4, -15, 1, 4, -33, 6,
    -26, -9, -37, 4, -23, -10, -1, -11, -3, -38, 7, 1, -23, -7, -25, 16, -14, 41, 26, 31, 54, 34, 49, 40,
    33, 26, 42, 21, 16, 15, 12, 23, 44, 23, 29, 27, 27, 20, 55, 38, 18, 27, -3, 32, 20, 10, 23, 40,
    16, 37, 18, 17, 29, 13, 11, 29, 5, -5, -1, -23, -7, -6, 12, -17, -20, -33, -24, -25, -4, 1, -15, 46,
    20, 37, 33, 70, 37, 77, 18, 53, 57, 13, 59, 49, 33, 42, 42, 30, 40, 36, 26, 23, 8, -11, 3, -9,
    -16, 1, -26, -32, -24, -8, -53, -3, 8, 1, -8, -1, -5, 3, -36, 25, -14, 18, 8, 15, 19, -9, 3, -2,
    6, 4, -4, -9, 6, -6, 10, -16, 12, 3, 22, -1, 6, -5, 3, 18, -6, 10, 8, 34, 5, 31, 13, 24,
    25, 12, 15, 18, 22, -6, 29, 4, 12, 38, -22, -4, -19, 6, 41, -2, 19, 4, 0, 0, -30, -14, -30, 3,
    -13, -3, -23, -17, -11, -19, -1, -25, -16, -6, -20, -28, -27, -34, -35, -33, -42, -51, -8, -43, -25, -33, -25, 11,
    -2, 6, 11, -5, -17, -8, -25, -36, -8, -38, -24, -50, -44, -39, -48, -12, -36, -20, -54, -12, -68, -55, -19, -89,
    -54, -77, -85, -45, -73, -62, -59, -38, -51, -27, -35, -74, -52, -95, -58, -89, -72, -79, -61, -56, -84, -42, -51, -58,
    -38, -28, -63, -20, -38, -20, -12, 4, 0, 26, 12, -10, 30, 4, 33, 39, 11, 1, -8, -28, -4, -62, -52, -44,
    -58, -47, -74, -39, -66, 6, -34, 8, 11, 3, 16, 11, 3, 9, -1, 17, 37, 23, 36, 36, 41, 14, 35, 17,
    42, 35, 25, 34, -4, 15, -8, 13, -13, 13, 10, 10, -2, -40, 3, -37, -7, 11, 20, -26, 30, 15, 32, 38,
    24, 74, 44, 48, 53, 49, 38, 30, 47, 28, 9, 32, 3, 60, 18, 17, 41, 7, 15, -2, 11, 30, 31, 18,
    51, 40, 48, 58, 79, 48, 87, 69, 95, 94, 58, 91, 65, 99, 51, 61, 44, 62, 48, 10, 35, 43, 23, 19,
    13, 27, 18, 18, 20, 27, 32, 35, 37, 28, 7, 23, 5, 15, 12, -3, 7, 16, 27, -3, -6, -14, -5, -13,
    -31, -17, 4, -52, 0, -43, -25, -32, -31, -22, -29, -6, -13, 12, -24, -11, -15, 18, 12, -11, -26, 0, -31, -13,
    -20, -30, -15, -38, -26, -43, -58, -36, -31, -67, -67, -44, -86, -81, -71, -86, -75, -67, -85, -51, -96, -71, -31, -74,
    -76, -54, -53, -43, 5, -50, 20, -18, -31, 3, -19, -23, -11, 16, -54, -12, -28, -35, -15, -53, 0, -22, -32, -37,
    0, -71, -23, -31, -48, -29, -40, -37, -11, -46, -16, -25, -16, -1, -25, -18, -2, 6, -12, 21, 39, 25, 20, 34,
    13, 37, 14, 39, 37, 49, 40, 52, 41, 41, 40, 49, 40, 50, 54, 23, 72, 31, 76, 30, 52, 41, 31, 46,
    -3, 47, 4, 49, 14, 36, 64, 32, 53, 18, 22, 20, 18, 25, 48, 43, 29, 43, 21, 16, 34, 19, 33, 47,
    14, 58, 25, 31, 29, 27, 43, 16, 44, 6, 10, 10, 32, 42, 62, 58, 39, 32, 12, 22, 36, 22, 29, 14,
    10, 10, -12, 3, 13, 11, 1, 3, 2, -11, -14, -9, -11, -15, -47, -31, -41, -41, -29, -36, -16, -15, -27, 14,
    -28, -15, 3, -9, 2, -10, -6, -33, -33, -59, -24, -72, -45, -44, -59, -45, -70, -53, -57, -47, -56, -34, -23, -29,
    -51, -52, -80, -24, -74, -50, -55, -42, -70, -56, -28, -55, -33, -38, -29, -34, -74, -66, -68, -77, -93, -42, -71, -67,
    -67, -57, -48, -53, -29, -6, -32, -20, -34, -5, -35, -10, -15, -29, -18, -24, -2, -18, -11, 7, 2, 0, 14, 6,
    -11, -19, 11, -18, 14, -34, 24, -30, 2, -3, -2, 7, -18, -4, -9, 2, 11, 21, 10, 6, 27, 17, 14, 29,
    4, 28, 13, 1, 13, -2, 16, -4, 20, -2, 12, 7, 9, 26, 7, 65, 10, 31, 21, 34, 7, 24, 50, 34,
    47, 35, 59, 58, 29, 58, 33, 52, 34, 68, 15, 49, 32, 20, 61, 15, 45, 40, 24, 19, 30, 21, 13, 9,
    17, 12, 26, -17, 16, -4, 15, 33, 19, 26, 0, 27, 19, 42, 19, 33, 12, 39, 21, 16, 23, 5, 24, 29,
    26, 13, 21, 29, 16, 16, 24, -4, 34, -1, 15, -6, 12, 1, 11, 23, -12, 50, -17, 24, 28, 7, 24, 30,
    2, -4, 7, -14, 5, 7, -25, -5, -18, -43, -8, -14, -6, -14, 1, -27, -28, -8, -35, -3, -38, -14, -44, -34,
    -64, -55, -48, -44, -54, -47, -63, -47, -9, -12, -39, -2, -17, -50, 11, -20, -15, 9, -6, -23, -10, -16, -9, 4,
    -32, 11, -14, -52, -39, -25, -13, -8, -17, 3, -29, -17, -52, -5, -25, -9, -24, -22, -26, -82, -20, -23, -27, -20,
    -25, -23, -12, -17, -18, -4, 2, -19, -1, -17, -33, -2, -48, -39, -20, 6, 15, 31, 25, 11, 21, 22, 48, 47,
    29, 63, 32, 28, 18, -3, 13, 18, 14, 41, 41, -5, 9, 31, 4, 42, 38, 37, 21, 25, -17, 18, -9, 13,
    38, 16, 34, 19, 54, 15, 38, 55, 71, 39, 66, 59, 51, 76, 49, 85, 78, 52, 43, 67, 46, 47, 30, 43,
    27, 42, 35, 31, 41, 12, 27, 34, 10, 33, 26, 34, 34, 20, 6, 45, 4, 26, 14, 29, 9, 13, 18, 15,
    27, 29, -16, 30, -1, -29, 6, -22, 18, -19, 0, -34, 20, -47, 11, 11, -34, 16, 6, -12, 7, 10, -11, 1,
    3, -28, -4, -3, -38, 26, -24, -8, -2, -17, 2, -13, 30, -54, 16, -35, -19, -41, -42, -53, -56, -28, -96, -29,
    -72, -58, -56, -52, -64, -41, -48, -44, -33, -36, -25, -46, -35, -51, -52, -36, -24, -38, 6, -27, -13, -5, -28, -10,
    -32, -6, -37, -18, -79, -24, -56, -23, -40, -18, -1, -40, -28, -12, -34, -47, -5, -36, -8, 5, -45, -4, -25, -36,
    -13, 1, -27, 13, -4, 13, 21, 7, 25, 17, 26, 28, 36, 32, 28, 35, 45, 27, 33, 33, 31, 30, 38, 35,
    32, 36, 45, 43, 49, 23, 59, 44, 44, 42, 39, 44, 29, 41, 37, 34, 46, 33, 35, 29, 1, 16, 16, 21,
    43, 11, 25, 46, -16, 17, 20, 2, 41, 45, 14, 54, 31, 7, 60, 29, 18, 62, 28, 21, 54, 10, 26, 44,
    42, 25, 64, 8, 19, 24, 5, 2, 25, 7, 8, 29, 12, 7, 17, -1, 2, 32, 7, 44, 24, 42, 18, 24,
    30, -18, 32, -24, -6, -5, -8, -2, -5, 7, -42, 16, -34, -10, -39, -23, -39, -29, -34, -37, -28, -55, -30, -44,
    -51, -46, -48, -71, -25, -52, -9, -18, -24, 5, -19, -31, -18, -15, -38, -24, -37, -44, -43, -54, -47, -58, -48, -52,
    -56, -59, -77, -60, -77, -73, -71, -63, -94, -76, -89, -80, -88, -86, -88, -66, -74, -79, -61, -65, -63, -57, -64, -64,
    -60, -39, -42, -29, -43, -33, -27, -32, -14, -6, -40, -24, -32, -18, -23, -22, -12, -7, -20, -35, -9, -26, -9, -14,
    -8, -6, -27, -15, -9, -12, -8, -2, -7, -12, 5, -10, -1, -6, -1, -23, -22, -16, -37, 9, -9, -7, 6, -15,
    -25, 1, -5, -8, 6, -34, -1, -9, -20, 13, 0, -8, -10, 1, -25, -7, 3, -34, 9, -27, 6, -19, -9, 1,
    -2, 15, 10, 14, 26, 12, 32, 22, 24, 26, 29, 23, 31, 25, 0, 31, 24, 1, 12, 23, 24, 46, 20, 46,
    23, 52, 38, 47, 31, 11, 17, -4, 15, -13, 7, 23, 20, 27, 16, 18, -9, 30, 21, 37, 34, 27, 29, 36,
    25, 37, 27, 43, 48, 29, 43, 25, 35, 35, 48, 45, 35, 34, 36, 35, 54, 23, 40, 10, 23, 5, 14, 37,
    2, 38, 8, 38, 18, 38, 45, 31, 43, 26, 40, -1, 10, 14, -22, 23, -20, 13, -4, -10, 26, -13, 6, -7,
    -1, -15, 7, 8, -13, 23, -10, 6, -14, 1, -2, 3, -8, -22, 0, -19, 20, -22, 29, -1, 7, 8, 8, 5,
    19, -1, 12, 6, 1, 0, 10, -19, 19, 9, -2, 25, 3, 11, 13, 10, 12, -22, 21, -21, 8, -29, 1, -8,
    -40, 5, -24, -2, -21, -5, -19, -9, -15, -2, -10, 16, 5, -6, -5, -6, -2, 5, -5, 4, -8, -13, -8, 20,
    -2, 9, 13, -3, 26, 5, 34, 28, 12, 38, 20, 40, 1, 16, 10, 10, 12, 19, 4, 7, 10, 17, -19, 14,
    7, 9, 19, 7, 10, 1, 14, -13, 40, 10, 12, 27, -5, 18, 20, 4, 30, 39, 30, 4, 37, 0, 18, 41,
    -15, 21, 23, -15, 24, 0, 9, 17, 6, -9, 14, 11, -8, 20, 3, 10, 3, 11, 18, 24, 50, 31, 33, 30,
    28, 25, 36, 26, 20, 15, 27, 8, 28, -2, 19, 25, 6, 23, 52, -12, 45, 24, 33, 12, 28, 7, 28, 33,
    -4, 25, -22, -12, -8, -16, -4, 7, 42, 10, 53, 16, 66, 54, 67, 51, 48, 33, 54, 35, 44, 39, 30, 24,
    22, 20, 9, 27, 19, 8, 14, 11, -11, 4, -5, -13, -11, -23, -26, -21, -34, -36, -23, -24, -24, -15, -22, -32,
    -31, -17, -30, -16, -12, -39, -16, -25, -39, -20, -39, -31, -53, -12, -47, -25, -27, -40, 15, -38, -7, -26, -10, -16,
    -20, -4, -31, -18, -2, -34, -17, -22, -22, -35, -42, -33, -52, -18, -44, -31, -27, -26, -46, -37, -33, -69, -23, -76,
    -28, -42, -17, -34, -23, -16, -33, 8, -33, 2, 3, -9, -9, -16, -18, -24, -17, -33, -12, -22, -32, -16, -46, -15,
    -39, -25, -17, -39, -12, -48, -13, 5, -14, 14, -8, 19, -2, 18, -8, 21, -6, 0, 9, -26, -10, -22, 3, -41,
    -24, -35, -30, -24, -33, -24, -20, -26, -23, -22, -32, -23, -41, -33, -12, -40, -13, -31, -14, -29, -2, -6, -11, 6,
    -12, 4, -12, -6, 7, 0, 8, 6, 3, -2, 3, -12, -16, -2, 3, 8, -14, 28, -1, 5, 3, 11, -22, 8,
    -24, -22, -7, -23, -2, 4, -12, 10, 7, -4, 8, 16, -15, 18, 1, 2, 14, -8, -1, 12, -8, -8, -11, 1,
    -25, 4, -8, 7, -8, -6, 21, -4, 22, 8, 27, 28, 21, 17, 26, 18, 22, 21, 30, 22, 33, 16, 19, 31,
    6, 14, 31, 6, 23, 17, 18, 11, 8, 13, 23, 20, 5, 17, 11, 23, 9, 14, 6, 11, -12, 15, -9, -5,
    0, -15, -7, -7, -20, -6, -8, 6, -6, 2, -2, 6, 15, -11, 6, 5, -22, -1, -16, -11, -15, -26, -9, -21,
    -10, -23, -22, -17, -27, -22, -22, -15, -42, -34, -33, -61, -48, -67, -42, -43, -32, -23, -16, -19, -25, -8, -5, -10,
    -4, -24, -6, -25, -6, -4, -32, 6, -29, -23, -9, -28, -2, -20, 7, -31, 10, -11, 3, -6, -35, 11, -19, 3,
    -9, -6, 10, -7, 16, 11, 19, 32, 6, 29, -23, 6, -14, -13, 21, -10, 10, -5, 1, -17, 6, 4, 20, 18,
    1, 20, 22, 17, 26, 15, 22, 10, 23, 8, 24, 33, -1, 30, 8, 26, 20, 11, 18, 13, 15, 12, 19, 17,
    19, 48, 3, 33, 11, 32, 35, 46, 27, 52, 43, 36, 70, 43, 64, 63, 64, 69, 65, 54, 61, 76, 35, 75,
    63, 56, 59, 69, 49, 42, 52, 44, 46, 46, 51, 34, 43, 12, 27, 32, 34, 38, 50, 32, 47, 57, 18, 45,
    30, 27, 47, 21, 31, 15, 33, 1, 20, 14, 5, 22, -13, 23, 0, 22, 13, 23, 21, 3, 4, 8, -3, 11,
    -17, 6, -22, -9, -9, 1, -3, -5, 2, -26, 0, -38, -6, -1, -18, -8, -21, -20, -45, -10, -39, -27, -26, -47,
    -24, -14, -14, -30, -7, -18, -9, -20, -7, -28, -10, -19, -25, -14, -47, -22, -35, -26, -32, -26, -41, -42, -41, -37,
    -37, -39, -52, -43, -60, -37, -54, -39, -47, -40, -36, -46, -40, -13, -27, -50, -16, -35, -18, -34, -8, -27, -21, -44,
    -13, -38, -32, -18, -36, -8, -29, -10, -18, 6, -6, -1, 12, 8, 0, 8, -11, -3, -15, -25, -4, -13, -14, -6,
    -12, -26, 10, -12, -17, -4, -8, -11, -5, -4, -9, -21, 3, -14, -4, 6, 0, 17, -12, 13, -6, 12, 21, 9,
    4, 5, 10, 10, 22, -5, 14, 27, 10, 24, 4, 4, 15, 32, 19, 21, 16, 25, 1, 29, -1, 18, 9, 3,
    10, 6, 22, 1, 27, 24, 13, 23, 2, 15, -2, 6, 10, 7, 5, 5, 23, -10, 22, -3, 8, 18, -5, 17,
    -6, 15, 5, -12, -9, 4, 13, 38, -8, 35, 6, 22, 5, 41, 19, 34, 40, 14, 15, 16, 16, 10, 18, -2,
    22, 3, -1, 4, 8, -2, 36, 0, 25, 22, 27, 12, 9, 27, 8, 20, 23, -1, 13, -1, -3, 22, -7, 22,
    12, 29, 14, -2, 30, -10, 9, -2, 19, -12, 29, -13, 20, 26, 22, 38, 42, 23, 47, 47, 41, 47, 35, 39,
    21, 56, 8, 41, 19, 26, 26, 3, 18, 26, 17, 33, 16, 40, 21, 34, 30, 33, 28, 8, 43, -6, 13, 24,
    -9, 11, -1, -23, -17, -15, -19, -17, -4, -10, 7, -8, 1, -1, -4, 9, 14, 2, 1, -9, -13, -27, -2, -23,
    -19, -30, -24, -38, -19, -11, -10, -22, -25, -19, -29, -19, -10, -17, -25, -12, 2, -27, -6, -6, 3, 13, -8, 2,
    16, -3, 21, 28, 1, 24, 1, -4, 7, 11, -3, 15, -14, 8, 7, -8, 15, -10, 21, -4, -15, 11, -4, 3,
    -9, -13, -16, 3, -7, -7, 25, -4, -1, -1, 0, -12, 28, 3, 3, -8, -2, -11, -9, -10, -23, 9, -22, -14,
    -20, -25, -1, -23, 1, -14, -8, -9, -17, -15, 8, -23, -19, 3, -26, 9, -12, -12, -11, -20, -21, -21, -11, -35,
    -25, -22, -26, -21, -22, -19, -24, -2, -13, -13, -11, -7, -11, 4, -21, 0, -16, -19, -22, -20, -40, -17, -26, -35,
    -31, -47, -36, -40, -27, -32, -5, -26, -28, -22, -25, -6, 2, 7, -1, -4, -10, -6, -8, -13, -14, -6, -25, -19,
    -44, -37, -21, -58, -41, -23, -41, -25, -26, -45, -23, -26, -16, -1, 4, -26, 21, -7, 2, 13, -19, 4, -1, -16,
    2, 3, -17, -15, -6, -13, 1, 1, -13, 2, -9, -28, -9, -1, -16, 0, -2, -30, 22, -28, 7, -11, 1, -9,
    9, -3, -3, 4, -31, 8, -18, 4, -1, -2, -15, 14, -28, 5, -2, -3, -10, 2, 2, -3, -7, 3, -2, 8,
    2, 17, 6, 24, 2, -1, -1, -1, 13, -1, 11, -11, 15, -20, 16, -1, -5, 8, 0, 9, -4, -18, -19, -27,
    -31, -9, -20, -16, -9, -34, -41, -17, -35, -1, -33, -4, -27, -19, -11, -22, -20, -1, 16, -24, 5, -4, 13, -11,
    -29, 3, -10, -16, -10, -10, -28, -7, 10, 18, 38, 21, 42, 28, 37, 27, 30, 58, 42, 27, 42, -1, 13, 14,
    22, 27, 38, 25, 12, 31, 15, 33, 34, 17, 34, 22, 9, 20, 15, 22, 24, 41, 51, 52, 33, 58, 35, 51,
    66, 41, 85, 46, 41, 59, 30, 45, 59, 41, 59, 43, 48, 45, 35, 36, 34, 31, 38, 18, 35, 16, 11, 27,
    2, 34, 3, 9, 13, 16, 5, 19, 15, 17, 5, 18, 8, 19, 0, 18, -6, -7, 2, 6, 2, -1, -3, -1,
    6, -26, 30, 9, -11, 6, 0, 2, 15, 12, 28, 23, 23, 16, 23, 14, 22, 39, 36, 10, 18, 15, 13, 10,
    20, 10, 4, 18, -12, 3, -5, 12, -9, 9, 5, -7, -31, -30, -39, -21, -29, -40, 3, -26, -45, -22, -69, -43,
    -48, -63, -26, -39, -43, -36, -33, -30, -24, -18, -24, 11, -26, 0, -24, -10, -1, -15, -10, 1, -20, -32, -12, -14,
    -31, 10, -11, -11, 2, -6, 4, -17, -18, -12, -13, -27, -14, -17, -34, -14, -28, -35, -20, -16, -33, 7, -30, -9,
    -5, -15, -8, -22, -12, 15, -13, -11, -15, -22, -18, -17, -21, -13, -21, -19, -15, -25, -9, -15, 2, -27, 15, -32,
    0, 10, -25, 0, -26, -3, -2, 7, 11, 3, 5, 15, -9, -4, 9, 7, 18, 41, 18, 27, 34, -2, 35, 18,
    24, 27, -1, 4, 12, -9, -1, 4, -13, 3, -3, -10, -1, -22, 9, 7, -8, -5, 2, -2, -8, 9, -16, 14,
    4, -1, 7, -8, 16, -4, -4, 3, 0, -1, -3, 14, -24, 7, -15, -9, 4, -24, 12, -20, -9, -7, -12, -12,
    -25, -1, -11, -7, -8, 7, 5, -18, -4, -12, -21, 4, -21, -21, -2, -28, -27, -33, -40, -16, -30, -39, -21, -31,
    -43, -23, -42, -34, -42, -44, -16, -58, -3, -24, -15, -21, -14, -13, -17, 15, -23, 17, -21, -7, 9, -6, -4, 13,
    -2, 10, 15, -17, 13, 0, -6, 22, -2, 8, -13, 9, -2, -5, 3, 14, 4, -2, 31, -11, 5, 14, -14, 27,
    -15, 6, 12, -3, 8, -2, 5, -3, -1, 3, 12, 6, 0, 23, 6, 21, 19, 10, 19, 31, 22, 25, 24, -1,
    19, 7, 19, 14, 17, 26, 32, 6, 23, 19, 29, 16, 30, 30, 42, 18, 24, 49, 23, 35, 36, 40, 32, 20,
    34, 23, 35, 9, 53, 24, 30, 13, 16, 1, 25, 5, 13, 0, -5, -2, 14, 12, 22, 24, 28, 12, 22, 10,
    1, 39, -7, 33, -11, -1, 9, -6, 8, -10, -3, 0, 6, -6, 32, 11, 10, 15, 2, 11, 17, -4, 21, 15,
    1, -14, -5, -7, 5, -9, -2, 0, -20, -3, 0, 7, 15, 12, 29, 17, 17, 22, 26, 25, 34, 24, 20, 38,
    31, 40, 18, 43, 6, 25, 17, 1, 9, 26, 5, 28, -2, 20, 19, 20, 26, 14, 15, -2, 3, 8, -22, 2,
    -11, -6, -15, -23, -6, -29, -20, -19, -21, -14, -12, -37, -13, -22, -8, -30, 6, -19, -6, -12, -15, -13, -25, -21,
    -24, -19, -38, -6, -24, -21, -23, -33, -13, -30, -9, -15, -15, -13, -18, -18, -18, -29, -25, -36, -28, -41, -29, -30,
    -30, -27, -25, -30, -31, -32, -36, -31, -34, -38, -28, -45, -25, -26, -30, -16, -35, -24, -34, -12, -22, -17, -26, -20,
    -20, -23, -6, -12, 4, -13, -7, -16, 15, -14, -3, -12, -27, -12, -24, -8, -4, -35, 0, -18, -14, 5, -8, 15,
    -3, -2, -10, 4, -27, -8, -24, -17, -9, -8, -8, -2, -11, -3, -3, -8, -14, -15, -6, -14, -19, -31, -31, -51,
    -25, -55, -31, -50, -44, -39, -40, -31, -15, -35, -9, -9, -38, -7, -24, -22, -5, -14, -29, -2, -38, -31, -16, -38,
    -29, -24, -40, -34, -24, -31, -13, -36, -20, -27, -29, -18, -16, -22, -27, -25, -29, -20, -46, -24, -21, -34, -17, -32,
    -19, -20, -25, -3, -24, -9, 9, -18, 0, -7, 5, 2, 8, 2, 5, 8, -1, 13, 6, 2, 14, -5, -1, 6,
    -9, 16, 8, 4, 21, -5, 14, 29, -1, 36, 3, 10, 17, 9, 19, 40, 29, 10, 34, -5, 30, 14, 34, 25,
    43, 13, 47, 22, 25, 37, 30, 37, 28, 23, 25, 21, 17, 23, 18, 28, 26, 27, 9, 32, 31, 26, 27, 22,
    27, 32, 16, 26, 18, 20, 28, 16, 22, 20, 39, 23, 26, 12, 22, -6, 3, 4, -16, -9, -19, -3, -1, -8,
    -3, 3, -5, 4, 13, 4, 20, 16, 8, 2, 3, 0, 6, -1, 7, -4, 5, 10, -11, 13, -6, 9, -21, 16,
    -23, -3, 5, -16, 7, 4, -9, 4, -10, -1, -9, 6, -12, 9, -25, 6, -11, -11, -12, 3, -11, -8, -8, -4,
    -7, -3, 4, -4, -12, -21, -5, -14, -10, -9, -5, -10, -15, -8, -20, 1, -25, -17, -1, -13, -18, -15, -25, -5,
    -17, -17, 0, -17, -11, -18, 1, -12, -2, -6, -12, -27, 6, -19, 12, -22, 18, -19, 11, 7, 7, 25, 7, 9,
    25, 15, 8, 21, -3, 20, 13, 6, 14, 0, 13, 4, 10, 9, 1, 12, 5, 16, 1, 10, 19, -8, 28, -18,
    20, -12, 5, -3, 17, 5, 16, 19, -1, 30, 12, 10, 9, 31, 3, 28, 10, 15, 12, 14, 0, 9, 5, 7,
    12, 16, -1, 6, 8, -2, -6, -16, 12, 0, 2, -9, 0, -5, 20, -9, 25, 12, 10, 24, 17, 26, 22, 26,
    19, 23, 18, 9, 17, 20, 21, 30, 24, 36, 27, 17, 16, 26, 9, 17, 9, 12, 10, 17, 5, 6, 15, 10,
    -1, 7, -14, -3, -17, -24, -17, -25, -21, -18, -12, -12, -6, -9, 12, -13, -7, -3, 2, 5, -3, 3, -16, 0,
    -38, -2, -13, -1, 2, 2, 16, -3, -8, 8, -22, -10, -8, -11, 7, 10, -9, 17, 21, -6, 38, -7, 17, 23,
    -2, 3, 8, 7, 2, -2, -4, 12, 8, 6, 16, 8, 29, 18, 2, 25, 3, 22, 13, 32, 9, 23, 15, 29,
    23, 17, 25, 30, 27, 26, 13, 11, 31, 3, 30, 4, 18, 21, 22, 15, 37, 15, 46, 31, 42, 45, 50, 38,
    51, 39, 56, 40, 46, 48, 35, 53, 31, 47, 32, 28, 41, 23, 33, 28, 32, 28, 30, 13, 45, 18, 46, 22,
    39, 29, 26, 29, 32, 27, 21, 37, 17, 32, 16, 24, 34, 15, 14, 15, 24, 27, 23, 31, 18, 19, 3, 10,
    2, 23, 9, 12, -3, 6, 3, -7, -4, 1, -5, -20, -23, -7, -28, -17, -23, -16, -15, -24, -21, -17, -28, -17,
    -12, -26, -10, -29, -26, -21, -39, -27, -43, -30, -21, -18, -33, -20, -20, -25, -24, -24, -19, -16, -29, -18, -21, -31,
    -27, -34, -37, -38, -37, -19, -44, -30, -36, -24, -23, -21, -38, -15, -30, -29, -28, -44, -16, -40, -33, -44, -26, -32,
    -26, -38, -35, -44, -23, -49, -18, -44, -25, -36, -34, -33, -37, -29, -31, -31, -35, -23, -35, -10, -43, -10, -39, -18,
    -50, -33, -47, -30, -42, -28, -34, -37, -28, -33, -16, -23, -10, -31, -6, -26, -7, -19, -3, -9, -1, -15, -11, -21,
    -10, -25, -21, -20, -28, -18, -18, -3, -5, 1, -11, 12, 1, 9, -2, 1, -1, 4, 4, 0, -3, -1, 3, -21,
    18, -15, 9, 3, -7, 2, 3, -11, -4, -3, -1, -3, 6, -2, -3, 2, 0, 17, 10, 13, 12, 16, 7, 14,
    13, 12, 7, 5, 8, 13, 10, 12, 12, 13, 8, 15, 16, 14, 22, 11, 19, 7, 2, 16, -1, 28, 8, 14,
    15, 2, 18, 11, 18, 25, 14, 30, 15, 17, 17, 10, 24, 15, 21, 13, 20, 18, 16, 20, 25, 23, 28, 28,
    16, 36, 17, 31, 18, 12, 14, 21, 12, 11, 8, 5, 13, 13, 6, 7, 15, 17, 10, 13, 24, 15, 24, 9,
    23, 2, 29, 8, 24, 24, 20, 26, 21, 25, 28, 20, 22, 41, 17, 36, 30, 36, 33, 16, 14, 28, 6, 19,
    10, 13, 31, 4, 15, 2, 20, -3, 15, -7, -5, -1, -6, -7, -3, -4, -6, -10, -5, -7, 17, 5, 5, 10,
    3, 10, 16, 28, 0, 20, 0, 17, 2, 7, 6, 18, 9, -4, 14, -1, 4, 17, -7, 31, -12, 7, 0, -4,
    11, 6, -9, 16, -2, 22, 19, 13, 14, 9, 3, 11, 13, -9, 22, -4, -4, -5, -12, -3, -1, -1, -4, -6,
    17, 12, 3, 5, 12, 4, 5, 2, 13, -4, -1, -11, -8, -11, -8, 7, -6, 8, 6, -1, 11, 0, -1, 8,
    4, 5, 6, 9, 8, 15, 1, 6, 11, -2, 13, 11, 3, 9, 6, -11, 6, -17, 14, -18, 11, -13, -4, -2,
    -12, -1, -14, -6, -9, -4, -16, -10, -26, -22, -6, -22, -11, -1, -25, 9, -29, -21, -34, -19, -33, -21, -33, -21,
    -28, -32, -29, -27, -25, -18, -26, -25, -31, -20, -36, -23, -39, -28, -37, -39, -11, -55, -25, -44, -31, -34, -40, -32,
    -21, -40, -32, -35, -27, -24, -27, -30, -22, -32, -22, -39, -27, -33, -28, -24, -28, -20, -21, -21, -15, -25, -20, -25,
    -25, -28, -22, -42, -19, -22, -25, -12, -37, -9, -38, -8, -27, 4, -11, -10, -6, -9, -8, -7, -13, 2, -5, -1,
    2, -7, 12, -4, -6, -11, -5, -13, -10, -12, -13, -18, -17, -17, -11, -16, -18, -12, -25, -19, -25, -27, -30, -6,
    -27, -26, -28, -20, -26, -35, -36, -28, -39, -21, -25, -36, -12, -20, -23, -7, -27, -17, -12, -24, -17, -9, -10, 6,
    -10, -10, -10, 7, -5, -8, -2, -15, 3, -21, -2, -28, 2, -5, -10, -4, 3, -10, -3, 9, -7, -16, -10, -9,
    1, -2, -1, 7, 18, 2, 14, 9, -3, 2, -8, -6, 6, -1, 9, -1, 14, 14, 24, 5, 23, 9, 25, 10,
    24, 3, 15, 5, 10, 10, -7, 9, -2, -7, 11, 7, 3, 14, 10, 25, 9, 28, 20, 23, 9, 20, 19, 17,
    27, 35, 25, 19, 17, 17, 35, 16, 25, 23, 9, 34, -1, 26, 21, 18, 12, 14, 17, 20, 18, 22, 15, 26,
    21, 25, 24, 16, 26, 30, 16, 16, 12, 9, 24, 1, 32, 4, 11, 10, 15, 22, 15, 20, 20, 32, 22, 27,
    13, 21, 31, 26, 27, 30, 8, 19, 13, 9, 19, 18, 21, 8, 22, 6, 16, 16, 5, 19, -3, 16, 28, 12,
    11, 18, 10, 23, 15, 16, 23, 31, 24, 22, 16, 28, 7, 29, 16, 22, 28, 21, 33, 8, 33, 17, 44, 19,
    36, 33, 31, 41, 38, 26, 33, 13, 38, 3, 13, 20, 2, 37, -11, 40, -11, 25, 14, 24, 26, 23, 17, 24,
    17, 15, 24, 14, 11, 4, 14, 17, 4, 17, -2, 11, 14, -7, 23, -14, 19, 4, 11, 7, 14, 20, 17, 14,
    17, 7, 9, 11, 0, 9, 8, 9, 4, 5, -4, 1, 20, -1, 13, 9, 14, 6, -2, -2, -14, -4, -18, -19,
    -5, -17, -10, -7, -33, -9, -29, -21, -29, -27, -14, -30, -29, -26, -25, -16, -11, -39, -26, -36, -34, -23, -31, -31,
    -23, -25, -20, -33, -11, -31, -12, -26, -20, -24, -31, -21, -37, -25, -18, -32, -28, -31, -47, -28, -46, -27, -27, -23,
    -34, -31, -25, -17, -22, -31, -30, -24, -29, -27, -18, -25, -25, -15, -31, -20, -23, -28, -19, -33, -29, -25, -30, -32,
    -33, -18, -35, -11, -46, -29, -30, -29, -13, -51, -18, -26, -32, -25, -35, -32, -44, -22, -48, -44, -48, -50, -32, -49,
    -38, -32, -23, -11, -15, -9, -11, 2, -8, 8, 4, -3, 17, 4, 11, 1, 14, -11, 17, -14, -10, -4, -2, 12,
    0, 4, 3, 16, 23, -5, 24, 9, 19, 18, 14, 6, 8, 21, 21, 14, 11, 12, 31, 13, 26, 18, 20, 27,
    14, 12, 16, 27, 25, 22, 13, 17, 17, 17, 9, 3, 14, 15, 21, 11, 17, 35, 14, 26, 18, 25, 37, 0,
    37, 0, 15, 29, 13, 20, 34, 31, 28, 26, 18, 31, 30, 33, 28, 34, 20, 26, 25, 15, 27, 19, 9, 20,
    8, 16, -1, 18, 13, 17, 10, 14, 17, 29, 8, 28, 4, 19, 20, 16, 26, 35, 30, 39, 35, 34, 48, 30,
    42, 49, 39, 37, 54, 34, 55, 23, 33, 31, 28, 29, 32, 24, 22, 20, 12, 14, 12, 15, 8, 14, 4, 12,
    15, 7, 13, 15, 7, 16, 19, 10, 10, 0, 3, 4, 9, 11, -5, 15, 7, -5, -2, -1, 8, 5, 7, 7,
    -3, -3, -17, -3, -15, -4, -25, -23, -16, -16, -21, -17, -19, -6, -29, -8, -12, -11, -6, -17, -15, -20, -15, -15,
    -7, -12, -17, -10, -9, -18, 0, -11, -10, 3, -3, 1, -16, -6, -11, 10, 1, 3, -2, 4, -13, -3, -1, 6,
    10, 11, 10, 11, 3, 1, -5, -4, 0, -11, -3, -12, -10, -26, -8, -43, -20, -18, -33, -9, -29, -22, -23, -19,
    -24, -21, -17, -28, -35, -15, -31, -36, -22, -46, -27, -29, -47, -26, -35, -36, -31, -40, -16, -46, -20, -44, -23, -37,
    -26, -32, -27, -24, -31, -25, -5, -35, -13, -14, -19, -1, -21, -19, -18, -4, -26, -11, -37, -11, -18, -26, -23, -29,
    -16, -30, -3, -17, 0, -9, -13, -4, -20, 1, -8, -5, -15, -13, -22, -15, -4, -15, -9, -9, -23, 4, -14, 14,
    -4, 4, -17, 6, -14, -1, -5, -17, 3, -15, -3, -17, -9, -8, 0, -4, 4, -3, -12, -10, -18, 0, -24, -27,
    -11, -28, -3, -25, -1, -16, -24, -9, -25, -11, -15, -18, -14, -24, -41, -18, -23, -3, -12, 3, -14, -8, -11, 5,
    -8, 3, -11, 10, -2, 0, 7, -10, 15, -12, 8, -7, -6, -4, -15, -4, -4, 0, -6, -4, 4, -5, 3, 4,
    -14, 11, -1, -1, 14, -1, 10, -12, 4, -7, 4, -10, 0, -7, 8, -11, -5, 7, 1, 11, 7, 22, 6, 17,
    0, 12, 6, 16, 8, 8, -1, 10, -9, 13, -10, -5, 12, -3, 15, 3, -2, -4, 8, 9, 20, -7, 24, 3,
    36, 14, 18, 20, 16, 10, 16, 11, 12, 23, 11, 26, 1, 35, -3, 47, 24, 17, 26, 29, 21, 32, 19, 35,
    29, 26, 16, 8, 10, 11, 22, -6, 21, -2, 26, -6, 15, 0, 7, 13, -1, 26, -15, 21, -2, -4, -7, 3,
    -2, 9, -3, 3, 1, 2, -14, -2, 2, -25, 13, -28, 11, -10, -3, -7, 1, 1, 15, 9, 12, 16, 10, 17,
    15, 12, 25, 21, 27, 17, 27, 35, 37, 41, 29, 38, 35, 37, 31, 23, 40, 30, 32, 29, 25, 22, 29, 18,
    25, 14, 27, 22, 30, 20, 13, 14, 19, 10, 21, 9, 18, 16, 17, 10, 14, 9, 31, 7, 30, -3, 17, 18,
    23, 22, 22, 28, 23, 27, 15, 23, 18, 24, 20, 13, 20, 20, 15, 21, 4, 7, 28, 21, 20, 13, 19, 20,
    24, 3, -3, 8, 18, -5, 17, -8, -7, 5, 9, -28, 6, -27, -9, -11, -14, -26, 3, -3, 1, -12, 4, -4,
    -13, 7, -3, 9, 2, 24, -19, 29, -18, 12, 5, 14, -9, 21, 7, 6, -3, 0, -3, 23, 4, -3, 6, 7,
    15, 4, 6, -3, 20, -3, 11, -3, 13, 3, 0, 8, -4, 10, 6, -6, 10, 6, 9, 13, 16, 0, -2, -5,
    9, 12, 0, 4, 5, 10, 3, -19, 7, -12, -7, -6, -7, -18, 0, -9, -10, -14, -5, -20, 20, 1, 11, 2,
    13, 9, 13, 3, -3, 16, 2, 0, -6, -17, -11, -8, 0, -7, -10, -16, -12, -22, -22, -16, -23, -6, -22, -17,
    -35, -28, -23, -33, -26, -33, -32, -22, -34, -31, -35, -34, -35, -35, -39, -36, -33, -37, -30, -50, -25, -54, -33, -31,
    -44, -38, -48, -50, -38, -51, -46, -46, -42, -28, -52, -38, -38, -35, -37, -34, -43, -23, -40, -21, -38, -38, -38, -39,
    -30, -47, -48, -46, -49, -39, -54, -26, -50, -34, -41, -38, -42, -33, -33, -24, -43, -25, -35, -30, -38, -39, -43, -47,
    -27, -49, -32, -33, -34, -25, -23, -37, -20, -32, -25, -17, -24, -17, -14, -11, -24, -17, -21, -11, -13, -14, -16, -12,
    -7, -11, -7, -24, -4, -2, -5, 18, 3, 11, 11, 6, 9, 6, 12, 10, 7, 18, 17, 16, 22, 19, 1, 18,
    10, 21, 12, 16, 10, 18, 25, 12, 23, 12, 31, 20, 41, 33, 29, 29, 10, 19, 7, 7, 29, 2, 34, 15,
    27, 21, 22, 26, 31, 33, 33, 35, 38, 35, 37, 32, 17, 31, 15, 24, 25, 34, 33, 24, 15, -1, 7, -7,
    5, 0, 4, -3, -4, -9, -11, -4, -3, 6, 0, 18, 10, 31, 13, 17, 24, 26, 20, 26, 14, 25, 32, 26,
    31, 25, 33, 34, 40, 33, 40, 35, 45, 24, 38, 34, 36, 47, 28, 32, 12, 24, 23, 27, 11, 20, 3, 3,
    -1, -9, 0, -8, 4, 3, 10, 12, 12, 25, 5, 17, 1, 14, 15, 5, 25, 14, 23, 4, -1, 9, 4, 17,
    8, 28, 23, 5, 30, 1, 24, 6, 27, 3, 29, 15, 19, 32, -10, 22, -29, 26, -2, 10, 15, 7, 26, 10,
    -7, 14, 8, 15, -8, 5, 6, 2, 36, -7, 28, 3, 27, 5, 11, 2, 13, 13, -3, 14, -11, 23, -1, -6,
    5, -1, 12, 5, -5, 5, 4, 21, -9, 20, 1, 25, 22, 9, 25, 5, 32, 17, 31, 8, 26, 11, 19, 11,
    6, 18, 24, 21, 23, 7, 24, 10, 23, 13, 20, 25, 11, 29, -3, 24, -1, 14, 3, 3, 16, -6, 15, -3,
    9, -4, -9, -1, -12, -3, 6, -1, 12, 3, -6, 13, 4, 1, 7, 3, 4, -10, -6, -20, -11, -2, -30, -6,
    -13, -16, 1, -25, -2, -11, -2, 0, -4, 8, -5, -4, -10, -14, -24, -21, -30, -12, -23, -14, -27, -25, -18, -29,
    -18, -22, -26, -15, -30, -18, -28, -29, -28, -37, -29, -44, -34, -42, -34, -27, -34, -40, -33, -26, -19, -15, -25, -4,
    -19, -10, -13, -11, -3, 5, -3, 11, -5, 7, 7, 1, -5, 5, 5, 0, -3, -12, 7, -8, -9, -13, -21, -10,
    -17, -17, -15, -23, -20, -6, -24, -16, -14, -19, -13, -22, -20, -29, -21, -22, -31, -8, -34, -20, -32, -21, -17, -15,
    -27, -16, -15, -21, -36, -30, -24, -20, -40, -21, -31, -27, -40, -41, -31, -30, -41, -28, -22, -35, -20, -35, -7, -24,
    -18, -28, -10, -26, -11, -20, -9, -3, -18, 3, -8, 9, 0, 3, 4, 10, -2, 3, -3, 9, 9, 20, 21, 27,
    17, 16, 11, 21, 19, 37, 18, 24, 17, 8, 24, 29, 6, 21, 4, 39, 16, 31, 20, 13, 36, 13, 22, 23,
    37, 35, 37, 25, 32, 35, 36, 33, 44, 51, 46, 48, 36, 40, 44, 35, 46, 33, 32, 49, 35, 48, 27, 39,
    39, 22, 35, 35, 33, 52, 24, 45, 28, 52, 43, 35, 43, 35, 42, 39, 36, 20, 31, 29, 20, 30, 30, 43,
    29, 33, 18, 22, 26, 15, 22, 1, 24, 19, 30, 10, -1, 25, 10, 21, 5, 2, 24, 22, 22, 23, 22, 26,
    30, 18, 35, 12, 30, 31, 31, 26, 14, 30, 23, 42, 12, 24, 28, 13, 42, 8, 14, 16, 15, 6, 12, 5,
    5, 14, -2, -10, -9, -11, -4, -3, -23, -22, -22, -14, -27, -31, -20, -14, -16, -20, -37, -26, -13, -15, -9, -11,
    -20, -14, -21, -24, -40, -25, -22, -22, -23, -30, -19, -31, -32, -37, -38, -34, -40, -21, -28, -18, -28, -33, -29, -28,
    -24, -34, -27, -29, -17, -47, -27, -48, -26, -36, -45, -35, -59, -37, -37, -40, -32, -45, -37, -34, -35, -40, -33, -25,
    -21, -28, -32, -25, -32, -27, -30, -33, -24, -26, -34, -29, -29, -43, -28, -50, -40, -53, -49, -43, -40, -43, -50, -57,
    -47, -51, -46, -56, -47, -56, -52, -53, -57, -55, -56, -58, -53, -61, -48, -54, -47, -46, -48, -57, -38, -46, -47, -21,
    -33, -21, -24, -29, -30, -11, -22, -12, -17, -10, -16, -2, -11, -4, -26, -17, -11, -33, 0, -18, -8, 2, -6, -10,
    -10, -6, -5, -6, -11, -2, 7, -2, 23, -4, 30, -2, 30, 12, 27, 29, 25, 31, 11, 33, 5, 23, 24, 36,
    29, 24, 26, 29, 30, 24, 34, 31, 43, 48, 35, 42, 41, 47, 46, 49, 44, 39, 62, 52, 60, 49, 55, 53,
    59, 70, 43, 83, 47, 58, 49, 67, 57, 72, 56, 56, 65, 42, 57, 54, 47, 47, 49, 34, 46, 43, 39, 33,
    29, 32, 40, 22, 48, 41, 32, 35, 29, 30, 28, 25, 37, 25, 26, 29, 9, 27, 10, 23, 13, 23, 19, 24,
    10, 14, 9, 4, 5, -5, -4, 2, -3, 8, -12, 0, -5, -8, -5, -9, -3, -14, -9, -14, -21, -27, -25, -38,
    -29, -42, -20, -17, -32, -21, -29, -22, -5, -4, -17, 9, -2, 15, 14, -3, 29, -4, 22, 14, 17, 18, 14, 26,
    22, 23, 22, 18, 16, 27, -9, 36, -1, 18, 2, -4, 5, -4, -1, 5, 5, 21, 5, 30, 4, 29, 15, 29,
    38, 20, 32, 22, 51, 33, 37, 24, 29, 37, 19, 27, 10, 22, 18, 23, 19, 19, 28, 16, 25, 7, 16, 10,
    15, 29, 8, 15, 15, 6, 17, 18, 24, 21, 12, 26, 13, 24, 14, 18, 19, 18, 20, 18, 11, 19, 24, 25,
    14, 16, 26, 23, 16, 18, 19, 13, 25, -3, 2, 11, -4, 9, -4, 0, 0, -9, 4, -21, -12, -12, -13, -10,
    -15, -13, -5, -10, -1, -14, -13, -14, -10, -21, -11, -31, -8, -16, -19, -26, -26, -26, -32, -27, -39, -24, -30, -37,
    -34, -48, -42, -51, -35, -47, -40, -38, -40, -28, -49, -47, -47, -62, -29, -73, -46, -53, -45, -40, -54, -48, -56, -45,
    -52, -43, -70, -54, -58, -63, -50, -64, -40, -53, -50, -48, -67, -41, -44, -51, -40, -64, -35, -62, -36, -58, -56, -47,
    -54, -64, -58, -65, -37, -39, -60, -28, -57, -46, -47, -36, -51, -46, -54, -51, -46, -50, -51, -45, -45, -48, -50, -61,
    -34, -50, -45, -55, -57, -42, -71, -48, -58, -29, -41, -47, -34, -39, -43, -33, -49, -35, -38, -39, -24, -34, -26, -21,
    -25, -15, -13, -28, 6, -5, -7, 2, -1, 10, 21, -4, 17, 16, 27, 22, 27, 11, 23, 20, 25, 21, 8, 26,
    -3, 37, 9, 15, 24, 13, 36, 23, 23, 28, 31, 22, 30, 30, 34, 42, 33, 41, 32, 48, 45, 44, 33, 39,
    47, 37, 48, 40, 42, 51, 55, 55, 69, 57, 75, 65, 64, 55, 63, 64, 66, 62, 68, 63, 69, 76, 59, 61,
    54, 65, 52, 73, 45, 81, 56, 72, 61, 54, 56, 45, 64, 45, 72, 41, 54, 49, 33, 38, 34, 31, 34, 40,
    25, 53, 29, 41, 42, 26, 44, 32, 36, 40, 25, 41, 32, 21, 49, 31, 31, 40, 31, 46, 29, 29, 31, 26,
    28, 15, 19, 3, 22, 15, 26, 17, 7, -4, 1, 8, 10, -9, 7, -4, 11, -10, 3, -2, -5, 0, -7, 1,
    -9, 9, -11, -1, -7, 5, -1, -11, -6, -7, -9, -17, -1, -8, 0, -14, -3, -20, -3, -16, -3, -15, -2, -30,
    -14, -25, -28, -19, -32, -17, -15, -20, -34, -27, -47, -38, -33, -56, -50, -58, -46, -59, -44, -69, -48, -57, -46, -39,
    -55, -35, -40, -37, -26, -50, -26, -34, -20, -37, -30, -31, -36, -32, -43, -43, -28, -49, -23, -42, -43, -44, -39, -43,
    -42, -52, -46, -58, -47, -67, -56, -57, -65, -56, -65, -64, -64, -69, -69, -62, -71, -70, -74, -82, -80, -71, -71, -85,
    -57, -79, -69, -62, -66, -34, -43, -32, -39, -38, -31, -16, -17, -10, -25, -4, -13, -11, -24, -8, -16, -17, -17, -11,
    -22, 10, -3, -4, 11, -7, 20, 1, 5, 4, 0, 3, -2, 2, -6, -8, -7, -15, 3, -19, -17, -13, -3, -9,
    1, -7, 5, 11, 9, 18, 29, 28, 37, 45, 26, 51, 33, 46, 35, 40, 51, 40, 43, 33, 34, 36, 18, 47,
    9, 32, 42, 25, 30, 31, 25, 31, 23, 1, 33, 2, 32, 22, 33, 11, 15, 39, 34, 36, 18, 27, 43, 49,
    39, 48, 53, 66, 58, 48, 48, 71, 62, 68, 50, 48, 67, 56, 84, 51, 68, 67, 77, 71, 79, 50, 74, 79,
    71, 91, 52, 83, 72, 62, 79, 69, 65, 70, 51, 86, 52, 58, 57, 53, 73, 61, 52, 58, 52, 73, 41, 60,
    46, 67, 73, 59, 56, 71, 55, 69, 47, 43, 45, 50, 58, 37, 42, 37, 49, 46, 36, 41, 38, 41, 50, 30,
    32, 25, 39, 43, 16, 53, 7, 59, 17, 27, 19, 22, 24, 8, 18, 19, 4, 20, 0, 9, -4, -7, -15, 5,
    -9, -20, -8, -18, -15, -11, -11, -22, -15, -25, -26, -20, -30, -18, -33, -26, -34, -43, -36, -39, -42, -40, -50, -36,
    -61, -42, -56, -47, -68, -63, -50, -65, -65, -64, -81, -62, -75, -68, -66, -70, -53, -73, -62, -73, -60, -74, -57, -67,
    -62, -57, -65, -42, -63, -58, -66, -66, -50, -79, -61, -62, -64, -45, -69, -56, -56, -44, -50, -46, -49, -38, -34, -48,
    -26, -55, -29, -48, -36, -43, -52, -37, -42, -41, -38, -45, -34, -45, -26, -36, -38, -25, -50, -36, -49, -46, -30, -45,
    -24, -36, -19, -26, -21, -35, -21, -27, -25, -35, -26, -12, -12, -21, -32, -19, -5, -23, -33, -18, -27, 2, -21, -12,
    3, -15, 15, -5, 15, 5, 13, 25, 17, 14, 23, 22, 21, 36, 9, 20, 24, 42, 4, 55, 26, 55, 49, 49,
    57, 47, 60, 51, 51, 45, 45, 46, 34, 43, 19, 31, 10, 30, 22, 38, 31, 32, 39, 36, 49, 29, 34, 51,
    34, 50, 36, 40, 45, 38, 54, 25, 49, 44, 32, 53, 27, 52, 35, 56, 52, 42, 63, 47, 78, 44, 67, 55,
    79, 55, 56, 66, 48, 72, 61, 47, 64, 51, 52, 67, 43, 60, 44, 74, 64, 55, 51, 56, 75, 59, 85, 39,
    84, 60, 70, 64, 73, 75, 85, 64, 60, 63, 65, 58, 58, 54, 51, 62, 52, 51, 32, 59, 37, 54, 45, 34,
    51, 48, 42, 43, 33, 47, 31, 35, 19, 28, 22, 25, 18, 18, 19, 11, 19, 11, 8, 5, -1, 7, 4, 6,
    -2, -12, -5, -25, -4, -28, -9, -5, -18, -15, -14, -30, -16, -32, -29, -31, -42, -27, -38, -35, -42, -35, -45, -42,
    -52, -56, -66, -47, -77, -64, -79, -72, -58, -76, -59, -73, -75, -67, -70, -74, -71, -75, -66, -65, -73, -82, -69, -86,
    -79, -77, -84, -81, -76, -90, -71, -89, -82, -79, -87, -76, -89, -78, -86, -77, -79, -76, -72, -65, -68, -71, -62, -65,
    -45, -76, -71, -71, -71, -54, -84, -62, -61, -59, -46, -56, -48, -32, -50, -45, -44, -45, -33, -40, -32, -32, -31, -21,
    -43, -36, -31, -27, -19, -28, -24, -26, -20, -21, -28, -30, -23, -26, -29, -27, -26, -26, -22, -26, -15, -26, -19, -25,
    -33, -28, -19, -40, -11, -31, -19, -19, -16, -37, -21, -40, -29, -19, -36, -11, -35, -10, -29, -24, -30, -17, -24, -28,
    -17, -38, -16, -24, -16, -23, -21, -34, -21, -35, -25, -36, -5, -38, -12, -34, -29, -10, -19, -8, -26, 0, -17, 9,
    -19, -9, 6, -13, 4, -18, 1, -4, -3, 0, -26, -7, -10, -5, -1, -5, 11, -4, 16, -15, -4, -5, 4, -9,
    -1, -14, 3, 5, -8, -12, -16, -13, -4, -13, 2, 17, 2, 34, -19, 30, 15, 36, 20, 18, 37, 31, 49, 18,
    29, 34, 28, 53, 40, 44, 46, 56, 47, 30, 43, 43, 50, 36, 66, 37, 53, 57, 52, 60, 60, 70, 69, 68,
    69, 64, 72, 69, 70, 65, 66, 60, 67, 60, 45, 69, 54, 62, 67, 48, 65, 58, 57, 62, 52, 62, 62, 61,
    50, 46, 43, 52, 57, 57, 61, 62, 58, 52, 52, 56, 61, 47, 55, 63, 61, 57, 43, 42, 52, 51, 58, 34,
    50, 43, 49, 32, 37, 22, 39, 30, 20, 19, 9, 37, 9, 27, -8, 6, 18, 7, 9, -13, 12, 7, 13, 11,
    8, 12, 29, 24, 22, 21, 10, 36, 19, 41, 26, 38, 26, 30, 34, 36, 34, 37, 39, 34, 46, 30, 34, 36,
    22, 33, 21, 30, 22, 18, 25, 11, 17, 2, 12, 10, 5, 5, -4, 14, -22, 1, -23, -13, -13, -17, -4, -33,
    -5, -13, -12, -15, -42, -20, -33, -17, -16, -19, -1, -9, 1, -22, -6, -22, -22, -16, -24, -11, -22, -4, -38, -14,
    -37, -23, -36, -37, -18, -13, -16, -26, -21, -23, -42, -26, -56, -43, -57, -50, -62, -51, -74, -41, -62, -51, -59, -47,
    -31, -45, -31, -54, -30, -36, -25, -33, -27, -28, -20, -32, -25, -37, -21, -39, -28, -43, -28, -27, -33, -12, -50, -23,
    -49, -27, -57, -27, -35, -24, -24, -41, -30, -43, -21, -35, -32, -32, -30, -36, -25, -35, -35, -39, -34, -32, -27, -37,
    -32, -29, -23, -8, -30, -11, -38, -11, -36, -16, -26, -22, -29, -29, -23, -36, -13, -29, -12, -36, -16, -25, -16, -14,
    -30, -5, -28, -12, -15, 2, -9, -10, -8, -18, -4, -12, 3, -1, 7, -7, -8, -3, -7, 5, 4, 9, 5, 10,
    -3, 10, 0, -3, 4, -6, -9, -5, -2, 0, -5, -9, 5, -10, 13, -15, 9, 0, -11, 12, -34, 11, -17, 11,
    -3, 13, 5, 3, 8, -3, -11, 1, -2, -13, 0, -19, -17, -8, -22, -5, -18, -2, 0, -2, -3, -4, 6, 7,
    -9, -3, -12, -8, -2, -12, -14, -12, -6, -10, -8, -4, -10, 0, 13, -14, 11, -11, 12, -3, 2, 13, -1, 16,
    7, -1, 11, -5, -1, 13, -6, 11, -5, 18, 17, 10, 15, 11, 14, 7, 4, 20, 8, 7, 21, 16, 28, 23,
    21, 29, 27, 37, 23, 27, 29, 32, 26, 35, 25, 35, 34, 31, 30, 35, 19, 37, 25, 21, 26, 16, 28, 37,
    7, 27, -2, 37, 11, 23, 15, 17, 28, 19, 25, 17, 32, 31, 25, 18, 37, 13, 35, 26, 27, 33, 14, 31,
    22, 25, 34, 28, 30, 26, 19, 24, 9, 26, 27, 28, 20, 13, 18, 19, 14, 20, 3, 28, 15, 26, 18, 17,
    17, 22, 27, 20, 25, 31, 30, 27, 30, 24, 24, 25, 23, 14, 11, 17, 19, 16, 8, 4, 1, 8, -9, -7,
    -15, 2, -14, 1, -17, -13, -13, -20, -14, -21, -22, -14, -10, -26, -31, -29, -34, -42, -41, -44, -49, -35, -41, -51,
    -50, -48, -55, -46, -61, -42, -65, -42, -55, -53, -44, -38, -43, -39, -29, -46, -30, -46, -44, -44, -35, -41, -53, -43,
    -60, -35, -50, -46, -60, -54, -44, -59, -47, -69, -64, -56, -52, -66, -57, -53, -43, -43, -51, -53, -50, -51, -36, -47,
    -40, -45, -33, -29, -42, -29, -31, -11, -19, -29, -8, -21, 4, -22, -13, -15, -1, 4, -14, 6, -13, 9, 0, 7,
    -9, 2, 7, 14, 4, 0, 5, 15, -4, 5, 3, 4, 15, -11, 13, -26, 11, -7, 14, -14, 3, -9, -2, -4,
    -7, -9, -1, 16, 5, 13, -5, 22, 10, 18, 16, 6, 26, 6, 22, 11, 25, 12, 31, 13, 34, 16, 25, 20,
    28, 23, 33, 15, 28, 24, 26, 31, 22, 28, 24, 41, 18, 49, 37, 35, 35, 19, 41, 30, 27, 29, 27, 28,
    42, 16, 45, 24, 33, 39, 32, 22, 30, 24, 26, 36, 17, 25, 24, 33, 29, 10, 20, 16, 20, 8, 22, 5,
    24, 22, 15, 11, 7, 8, -5, 8, -11, -4, -13, -10, -4, -6, -4, -1, -12, 5, -5, 9, 12, -3, 3, -2,
    5, 3, 15, 11, 16, 28, 28, 34, 27, 45, 39, 42, 53, 35, 49, 44, 47, 45, 36, 45, 22, 38, 22, 24,
    39, 29, 37, 33, 31, 26, 22, 24, 7, 6, 5, -2, 2, -1, -1, -7, 2, -7, -6, 9, -10, 7, 1, -10,
    12, -1, 2, 13, -2, 3, -5, 0, -16, 0, -15, -9, -2, -3, 11, -12, 6, -1, 10, -3, 6, 0, 6, 14,
    0, 8, 10, 7, 20, 11, 15, 5, 14, 18, 9, 25, 7, 27, 19, 15, -1, -5, 11, 0, 29, -4, 32, 9,
    28, 17, 4, 36, 12, 47, 18, 30, 21, 5, 25, 4, 10, -2, 10, 3, 2, 4, -8, 18, 2, 20, 5, 9,
    8, -8, 6, -13, -10, -4, -11, -10, -18, -8, -26, -10, -34, -23, -21, -32, -15, -34, -20, -37, -27, -19, -48, -9,
    -38, -31, -10, -43, -6, -40, -12, -18, -9, -11, -13, -19, -6, -19, -28, -14, -32, -21, -8, -25, 5, -22, -1, -3,
    6, 4, 5, 6, 22, 20, 2, 35, -6, 28, 16, 16, 26, 11, 23, 31, 28, 24, 22, 10, 30, 12, 29, 19,
    24, 30, 20, 17, 6, 24, 21, 21, 23, 30, 22, 27, 26, 9, 17, 18, 10, 17, 5, 10, 14, 7, 17, -6,
    10, -7, 21, -9, 25, -7, 16, 17, 7, 20, 6, 15, -5, 7, 5, 12, 11, -3, -5, 1, -8, -4, -4, 4,
    -10, -8, -11, 3, 4, 0, -6, 1, -10, -1, -19, -3, 10, 1, -5, -8, -10, 2, 0, -7, 19, -9, 8, 6,
    10, 6, 6, 9, 10, 1, 12, -1, 13, 2, 9, -1, 15, 6, 13, 8, 5, 15, -8, 9, -6, -2, 3, -2,
    4, -10, 4, -12, -11, -18, -11, -19, -20, -31, -35, -17, -20, -30, -26, -24, -26, -19, -32, -25, -22, -25, -30, -35,
    -34, -25, -33, -27, -33, -34, -38, -40, -41, -36, -38, -46, -18, -53, -25, -44, -33, -26, -40, -36, -32, -28, -25, -24,
    -38, -28, -33, -31, -20, -35, -13, -27, -32, -23, -20, -23, -10, -22, -13, -11, -10, -15, -11, -30, 5, -20, 3, -17,
    -12, -12, -17, -8, -26, 0, -29, 0, -26, -18, -20, -25, -14, -25, -13, -13, -8, -13, -12, -13, -5, -4, 1, 4,
    -6, 3, -2, 0, 8, 4, 12, 1, 8, 1, -9, 13, -9, 14, -3, 15, 10, 10, 10, 8, 2, 3, 12, 8,
    24, 6, 16, 11, 16, 4, 16, 3, 23, 10, 21, 3, 23, 8, 18, 1, 22, 19, 25, 4, 23, 10, 6, 21,
    3, 22, -2, 16, 1, 6, 13, -2, 25, 10, 10, 15, 9, 31, 6, 38, 1, 27, 7, 14, -1, 12, 10, 4,
    8, -9, 10, -18, 25, -13, 23, -10, 4, -2, 5, 20, -3, 18, -8, 13, -5, 6, 7, -2, 7, -2, 8, 0,
    -1, -21, -9, -23, -5, -15, -14, -12, 0, -8, -3, -28, -13, 0, -6, 7, -24, 16, -23, 11, -22, -15, -12, -16,
    9, -6, -17, -14, -17, -20, -17, -28, -27, -15, -37, -21, -47, -40, -41, -47, -25, -39, -35, -46, -39, -23, -35, -29,
    -37, -25, -26, -32, -11, -25, -20, -3, -16, -5, 6, 3, 13, 19, 11, 3, 8, 20, 19, 5, 10, 1, 6, 2,
    10, 5, -4, 29, -11, 10, -3, 6, 17, 22, 19, 16, 32, 18, 28, 9, 35, 20, 36, 24, 30, 27, 28, 29,
    30, 25, 37, 26, 41, 25, 27, 14, 35, 21, 39, 27, 25, 41, 5, 37, 10, 44, 22, 34, 25, 31, 43, 27,
    44, 23, 30, 30, 28, 40, 34, 27, 47, 15, 47, 25, 39, 39, 38, 44, 45, 35, 51, 17, 46, 29, 39, 35,
    25, 27, 31, 23, 40, 14, 30, 31, 16, 35, 1, 29, 12, 19, 15, -4, 19, 15, 10, 17, -1, 19, -3, 22,
    12, 14, -5, 24, 2, 7, 3, 1, 22, 11, 5, 16, -3, 10, 0, -3, 3, 10, -4, 13, -10, 11, 11, 10,
    2, -2, -12, 5, -5, 13, -14, -7, -1, -7, 8, -15, 7, -1, -13, 3, -24, 10, -9, -21, -3, -12, -20, -3,
    -24, -30, -1, -27, -13, -22, -27, -23, -16, -32, -38, -24, -22, -23, -34, -23, -39, -18, -28, -26, -43, -34, -56, -30,
    -41, -58, -40, -46, -32, -44, -48, -44, -42, -36, -57, -39, -40, -17, -20, -26, -17, -19, 1, -19, -28, -14, -10, -20,
    -6, -29, -16, -30, -15, -27, -34, -31, -35, -24, -32, -46, -33, -24, -24, -34, -34, -30, -24, -23, -23, -37, -17, -21,
    -21, -14, -32, 5, -27, 4, -16, 0, -25, -7, -27, -22, -16, -24, -23, -28, -27, -26, -29, -15, -29, -21, -27, -24,
    -2, -16, 5, -1, -15, -4, -27, 7, -20, 0, -23, -9, -10, -10, -20, -8, -8, -13, 0, -34, -11, -23, 6, -33,
    -10, -11, 10, 6, 16, 20, 39, 33, 37, 40, 31, 63, 55, 72, 64, 55, 56, 53, 56, 42, 53, 43, 45, 44,
    46, 52, 39, 37, 21, 38, 41, 25, 31, 35, 37, 43, 33, 26, 32, 34, 32, 44, 37, 56, 41, 57, 61, 55,
    73, 65, 77, 50, 60, 56, 59, 55, 54, 53, 52, 55, 45, 44, 32, 35, 39, 20, 38, 27, 23, 38, 18, 38,
    17, 33, 27, 27, 23, 36, 29, 44, 28, 31, 40, 30, 40, 30, 40, 28, 40, 21, 30, 29, 28, 24, 26, 13,
    28, 26, 13, 20, 9, 28, 14, 16, 9, 14, 29, 11, 9, 20, 13, 23, 8, 26, 7, 5, 1, -1, 7, -11,
    -1, -26, 0, -24, -17, -22, -35, -11, -52, -20, -49, -37, -40, -67, -38, -47, -53, -52, -68, -54, -56, -53, -45, -59,
    -48, -50, -56, -45, -58, -52, -51, -62, -43, -55, -50, -44, -55, -55, -41, -41, -42, -36, -36, -21, -43, -35, -44, -45,
    -49, -49, -53, -49, -55, -48, -64, -71, -74, -56, -66, -39, -76, -39, -68, -49, -52, -58, -43, -53, -46, -59, -43, -39,
    -45, -38, -56, -42, -35, -40, -31, -32, -35, -27, -38, -25, -30, -44, -9, -32, -12, -33, -26, -21, -23, -21, -36, -37,
    -15, -34, -21, -30, -32, -27, -37, -22, -30, -29, -16, -26, -21, -19, -21, -11, -21, -3, -14, -13, -1, -20, 8, -1,
    8, 5, -9, 10, -3, 10, 3, 12, 2, 23, 12, 22, 2, 24, 16, 17, 12, 11, 20, 21, 30, 8, 30, 9,
    40, 15, 24, 20, 16, 36, 9, 28, 14, 37, 40, 25, 34, 10, 41, 23, 37, 20, 35, 37, 30, 33, 17, 22,
    21, 27, 25, 18, 11, 14, 10, 6, 11, 6, 13, 9, 20, 3, 8, 9, 0, -8, 4, 3, 1, -9, -4, 6,
    -10, -2, -16, -17, 0, -17, -1, -23, -19, -7, -21, -12, -17, -27, -9, -24, -5, -27, -17, -22, -22, -11, -30, -32,
    -22, -23, -25, -18, -29, -25, -17, -23, -29, -23, -29, -12, -21, -30, -24, -28, -17, -19, -17, -30, -11, -26, -9, -30,
    -28, -30, -18, -23, -25, -10, -44, 10, -25, 6, -14, -17, -17, -19, -12, -9, -4, -1, 3, -3, 14, -5, 15, 4,
    12, -3, 6, 11, 13, 14, 11, 8, 16, 16, 15, 24, 11, 16, 17, 24, 17, 29, 33, 37, 38, 37, 51, 38,
    41, 47, 25, 52, 34, 46, 50, 32, 45, 29, 49, 22, 43, 25, 35, 36, 26, 45, 32, 50, 36, 49, 34, 43,
    38, 48, 47, 41, 45, 38, 67, 38, 49, 44, 47, 50, 54, 36, 48, 30, 45, 31, 35, 46, 37, 54, 34, 37,
    40, 37, 39, 44, 29, 28, 30, 23, 30, 27, 33, 23, 29, 32, 20, 35, 24, 32, 29, 22, 26, 16, 30, 24,
    27, 18, 23, 17, 23, 20, 7, 13, 17, 18, 12, 8, 3, 4, 6, -2, 4, -1, 1, -6, -1, 4, 0, 5,
    -6, -16, 2, -22, 8, -13, -13, -7, -28, 4, -28, -5, -32, -20, -26, -29, -38, -35, -39, -27, -39, -47, -42, -51,
    -54, -47, -53, -33, -44, -38, -39, -46, -24, -40, -21, -29, -45, -36, -38, -39, -29, -35, -38, -24, -42, -23, -33, -32,
    -27, -26, -26, -9, -31, -8, -24, -29, -20, -41, -23, -36, -22, -24, -24, -11, -9, -14, -2, -13, -6, 13, -6, 10,
    6, 8, 13, 2, -13, 4, -8, 6, 5, -2, 9, -18, 13, -13, -2, 7, 0, 15, 5, 12, 0, 19, 2, 8,
    1, 21, 30, 26, 16, 25, 27, 34, 40, 32, 41, 45, 37, 47, 36, 37, 38, 33, 32, 42, 45, 52, 45, 33,
    40, 46, 43, 54, 56, 50, 41, 49, 47, 48, 42, 33, 49, 34, 45, 35, 44, 46, 43, 58, 47, 51, 41, 43,
    47, 34, 54, 24, 48, 17, 30, 15, 21, 14, 13, -3, 0, 5, 0, 4, -21, -20, -27, -23, -36, -40, -41, -40,
    -33, -46, -34, -35, -27, -33, -32, -36, -29, -44, -30, -50, -26, -28, -37, -27, -35, -37, -33, -35, -48, -31, -45, -28,
    -45, -35, -42, -44, -51, -67, -63, -71, -52, -56, -61, -69, -60, -65, -60, -61, -65, -42, -56, -52, -39, -57, -31, -49,
    -51, -49, -60, -25, -54, -39, -61, -43, -35, -30, -32, -28, -27, -26, -34, -30, -23, -28, -17, -28, -14, -13, -12, -6,
    -10, -14, -6, -7, 7, -7, 7, 2, 0, 9, -12, -12, -27, -24, -12, -28, -24, -39, -26, -26, -31, -27, -22, -29,
    -24, -39, -31, -26, -10, -19, -4, -4, -13, -16, -3, -18, 18, 1, 12, 24, 14, 39, 24, 37, 33, 26, 37, 34,
    30, 24, 27, 29, 40, 21, 30, 32, 48, 35, 47, 31, 58, 43, 61, 43, 57, 54, 53, 49, 41, 44, 39, 35,
    22, 25, 31, 29, 32, 19, 18, 24, 21, 30, 13, 23, 17, 23, 27, 10, 30, 26, 44, 50, 42, 37, 42, 39,
    43, 44, 32, 51, 36, 47, 44, 38, 53, 40, 54, 28, 41, 36, 47, 38, 27, 42, 27, 45, 20, 18, 12, 21,
    21, 10, 0, 16, 2, 16, 8, -4, 25, -8, 9, -3, -4, 15, -4, 0, -7, 1, -8, 10, -7, 2, -7, -7,
    -1, -10, -6, -11, 1, -3, -4, -15, -5, -17, -15, -13, -25, -16, -21, -19, -21, -25, -31, -29, -34, -32, -31, -30,
    -29, -18, -25, -29, -13, -37, -5, -28, -15, -18, -29, -25, -24, -19, -23, -22, -36, -27, -41, -33, -39, -51, -14, -32,
    -16, -49, -35, -35, -30, -43, -27, -31, -26, -28, -29, -29, -38, -36, -25, -27, 0, -29, -14, -16, -16, -15, -20, -23,
    -3, -32, -15, -23, -10, -21, -17, -26, -18, -9, -30, -9, -26, -7, -16, -21, -5, -18, -14, -14, -10, -4, -19, -17,
    -5, 1, -15, -1, -8, 24, 12, 20, 1, 13, 12, 12, 23, 9, 42, 12, 40, 22, 33, 28, 27, 33, 17, 37,
    25, 46, 44, 30, 28, 35, 28, 44, 35, 42, 30, 34, 38, 25, 49, 17, 35, 35, 29, 37, 17, 24, 24, 30,
    28, 21, 15, 18, 28, 22, 15, 14, 18, 17, 19, 9, 19, 18, 22, 13, 26, 6, 19, 10, 6, 7, 10, 3,
    -6, 11, -13, 3, -15, -15, 0, -10, 2, -17, 15, -25, 18, -20, 11, 0, 2, 5, -2, 4, -2, 4, -16, -8,
    -8, -12, -15, -10, -13, -3, -5, -1, -25, -18, -25, -26, -22, -33, -25, -35, -40, -38, -41, -34, -36, -26, -48, -30,
    -43, -28, -49, -56, -47, -51, -33, -43, -37, -49, -45, -41, -49, -45, -55, -34, -41, -36, -33, -35, -32, -33, -41, -45,
    -33, -47, -34, -50, -39, -42, -48, -43, -49, -38, -51, -54, -52, -58, -56, -69, -57, -65, -47, -72, -45, -62, -46, -57,
    -62, -50, -52, -51, -51, -33, -48, -45, -40, -44, -32, -45, -46, -46, -40, -40, -46, -47, -39, -37, -32, -28, -22, -15,
    -7, -21, 5, -37, -3, -21, 5, 11, -7, 22, 8, 34, 19, 20, 40, 26, 48, 22, 46, 30, 43, 27, 36, 33,
    40, 52, 45, 72, 61, 63, 63, 63, 61, 58, 56, 46, 44, 43, 52, 41, 42, 39, 36, 59, 49, 61, 52, 61,
    82, 42, 82, 49, 57, 64, 38, 77, 47, 69, 57, 69, 52, 58, 47, 56, 54, 49, 61, 54, 72, 69, 68, 68,
    67, 70, 72, 75, 76, 67, 77, 59, 54, 66, 65, 59, 63, 54, 55, 62, 62, 64, 66, 55, 63, 76, 68, 69,
    61, 68, 63, 80, 58, 73, 62, 66, 75, 81, 72, 66, 75, 59, 90, 47, 72, 52, 54, 58, 44, 41, 52, 46,
    40, 37, 28, 43, 27, 49, 16, 23, 24, 8, 34, 5, 13, -1, 9, 3, 9, -12, -7, -8, -12, -16, -23, -12,
    -11, -16, -7, -32, -18, -18, -32, -24, -34, -38, -37, -39, -38, -54, -52, -36, -55, -46, -58, -50, -49, -43, -68, -65,
    -53, -39, -47, -49, -60, -46, -49, -60, -61, -68, -62, -54, -66, -60, -83, -52, -80, -66, -81, -66, -61, -77, -68, -94,
    -80, -76, -70, -64, -71, -68, -54, -65, -54, -52, -68, -44, -67, -39, -68, -50, -62, -57, -62, -71, -68, -41, -54, -34,
    -41, -47, -27, -45, -26, -40, -26, -33, -40, -34, -54, -42, -36, -31, -32, -25, -33, -2, -14, -11, -4, -23, -1, -13,
    -10, -2, -8, 7, -6, 0, 2, -3, 9, 4, 11, 11, 13, 19, 16, 25, 11, 19, 10, 20, 6, 25, 17, 35,
    34, 20, 41, 25, 40, 24, 30, 27, 28, 25, 34, 33, 22, 39, 13, 39, 20, 24, 34, 21, 32, 22, 23, 14,
    15, 23, 11, 7, 6, -5, -10, 4, -23, -11, -22, -34, 6, -47, 0, -28, -3, 12, -9, 16, -3, 9, 12, -7,
    17, -1, 25, 13, 17, 7, 26, 27, 22, 37, -7, 45, 28, 42, 28, 18, 20, 28, 32, 21, 17, 16, 20, 17,
    23, 17, 18, 15, 23, -4, 29, -18, 20, -3, 4, -5, -8, -5, -4, -1, 7, -12, 11, -12, 5, 2, -12, -3,
    -9, -8, -2, 8, -11, 14, -20, -5, -11, -25, 2, -14, -14, 4, -16, -5, -12, -17, -9, -12, -12, -15, -13, -11,
    -6, -17, 2, -5, -5, -1, -7, -2, 2, -6, 5, -4, 0, -9, 5, 8, 10, 22, -8, 8, 9, 16, 16, 11,
    8, 22, 19, 17, 2, 23, 8, 24, 13, 19, 10, 16, 14, 22, 18, 18, 28, 10, 16, 12, 11, 16, 23, 15,
    12, 23, 16, 31, 10, 23, 13, 27, 21, 28, 15, 19, 11, 11, 11, 6, 18, 12, 12, 13, 13, 4, 7, 5,
    -9, 2, -10, 0, -8, -14, -7, -4, 2, 3, 0, -3, -9, -7, -2, -16, -14, -25, -10, -12, -23, 3, -16, -4,
    -1, -13, -18, -14, -21, -14, -21, -20, -29, -23, -18, -14, -11, -21, -24, -10, -14, -9, -13, -8, 3, -9, -5, -18,
    -11, -1, -42, -13, -36, -20, -14, -18, -12, -29, -21, -33, -29, -42, -31, -42, -36, -46, -45, -38, -36, -30, -31, -51,
    -36, -52, -26, -34, -44, -52, -43, -50, -37, -49, -44, -40, -19, -37, -24, -26, -25, -15, -20, -23, -29, -25, -26, -13,
    -28, -18, -19, -17, -6, -27, -7, -12, 8, -1, -18, -12, -4, -5, 0, -22, 4, -12, 10, 7, -5, 8, -2, 19,
    -1, 6, -1, 11, 7, 3, -2, -1, 14, 4, 11, -13, 29, 5, 26, 6, 16, 23, 12, 31, 12, 13, 24, 10,
    25, 10, 19, 28, 18, 31, 5, 23, 24, 17, 44, 17, 33, 49, 33, 50, 44, 41, 50, 37, 47, 32, 38, 43,
    20, 52, 15, 44, 19, 36, 24, 23, 17, 6, 16, 16, 2, 15, 14, 4, 28, 3, 26, 6, 8, 26, -3, 24,
    7, 16, 29, 14, 12, 25, 16, 21, 10, 18, 29, 21, 41, -2, 38, 19, 35, 27, 34, 38, 34, 31, 19, 27,
    23, 23, 21, 27, 10, 22, 26, 15, 30, -4, 25, 8, 22, 8, 3, 15, 5, 2, -5, -7, -8, -10, -14, -7,
    0, -20, -4, -4, 0, 10, -12, 10, -7, 19, -5, -11, -7, -17, -8, -23, -22, -27, -23, -18, -42, -15, -39, -12,
    -39, -24, -27, -28, -24, -31, -23, -21, -22, -12, -16, -14, -21, -8, -9, -13, -5, -19, -8, -10, -4, -23, -16, -27,
    -6, -15, -11, -15, -25, -24, -25, -32, -24, -19, -29, -24, -41, -31, -30, -26, -31, -30, -45, -43, -49, -40, -37, -47,
    -38, -47, -48, -40, -49, -30, -48, -32, -46, -42, -31, -39, -30, -47, -36, -39, -45, -37, -37, -49, -38, -45, -32, -36,
    -47, -31, -29, -31, -31, -37, -39, -25, -32, -25, -28, -15, -21, -9, -17, -28, -10, -10, -1, -8, -10, -22, 0, -4,
    -4, -7, -10, -2, 6, -4, -3, -3, 2, -1, -6, -6, -3, -1, -2, -12, 0, -13, -4, -15, -9, -12, 0, -6,
    -1, 6, -8, 10, -16, 7, -12, 5, 7, 2, 7, 8, 6, 24, 5, 22, 18, 21, 12, 12, 6, -2, 18, 0,
    18, 5, 10, 26, -7, 17, 0, 9, 13, 9, 8, 1, 11, 25, 18, 22, 10, 4, 25, 23, 29, 23, 19, 32,
    32, 31, 40, 30, 45, 33, 46, 43, 43, 55, 45, 55, 49, 48, 56, 43, 63, 47, 50, 50, 25, 54, 32, 63,
    51, 58, 53, 59, 53, 56, 54, 56, 48, 45, 49, 32, 47, 33, 32, 45, 18, 40, 32, 45, 39, 31, 35, 33,
    43, 28, 38, 28, 40, 28, 29, 37, 35, 44, 17, 49, 30, 38, 35, 29, 35, 35, 25, 22, 12, 22, 11, 19,
    16, 10, 16, 9, 4, 22, 5, 20, 6, 17, -4, 17, -3, 21, 0, 11, 3, 2, 2, -16, 12, -21, -4, -20,
    -14, -13, -19, -7, -26, -3, -22, -18, -19, -12, -28, -21, -19, -31, -7, -26, -4, -15, -15, -13, -19, -3, -17, -14,
    -4, -11, 13, -8, 4, -5, -14, -7, -3, -7, -3, -5, -16, -5, -25, -9, -12, 0, -8, -10, -25, -10, -16, -10,
    -15, -22, -8, -11, -6, -20, -20, -13, -10, -10, -13, -29, -13, -9, -17, -11, -20, -16, -17, -12, -23, -18, -13, -5,
    -22, -14, -14, -15, 5, -11, 2, 9, -1, 21, 2, 11, 9, 2, 18, 17, 14, 12, 5, 22, 3, 13, 1, 14,
    12, 12, 7, 13, 16, 5, 12, -6, 7, 6, 18, 1, 8, 0, 1, -4, -6, 4, -10, -9, -5, -7, -20, -12,
    -24, 1, -12, -7, -30, -13, -18, -18, -26, -42, -17, -45, -13, -45, -23, -35, -26, -24, -38, -25, -45, -21, -36, -23,
    -30, -25, -22, -15, -19, -18, -16, -3, -23, -10, -34, -22, -25, -12, -26, -22, -2, -16, -1, -11, -6, -3, -10, 8,
    5, 10, 2, 3, -5, 15, -2, 14, 11, -2, 21, -10, 15, -5, 11, 18, 1, 17, 2, 17, 1, 12, 5, 6,
    7, 7, -6, 1, -3, 2, 8, -2, -3, -3, -2, 1, -18, -23, -19, -5, -8, -12, -19, -10, -13, -3, -2, -16,
    -13, -12, -5, -13, -1, -24, -5, 7, -13, 3, -23, -7, -14, -5, -12, -20, -15, -24, -31, -9, -21, -18, -19, -20,
    -8, -7, -9, -8, -10, 0, -10, -2, -11, 2, 8, 8, 5, -5, 18, 12, 13, 1, 8, 2, 10, 2, 5, 3,
    0, 23, 2, 20, 17, 8, 30, 21, 30, 30, 17, 29, 15, 27, 18, 24, 33, 32, 35, 28, 30, 29, 39, 29,
    37, 22, 24, 32, 22, 31, 21, 36, 31, 26, 13, 15, 16, 13, 12, 14, 13, 12, 14, 15, -2, 24, 6, 18,
    1, 5, 12, 4, 0, -8, -1, -1, -1, 4, -2, 0, 0, -14, -9, -13, -15, -9, -9, -8, -9, -7, -3, -9,
    -4, -7, 4, -10, -4, -17, -22, -8, -23, -12, -31, -14, -13, -1, 5, 3, 10, -3, -2, -6, 13, -4, 21, -10,
    24, -6, 15, 10, 17, 12, 0, 36, -1, 44, 10, 10, 4, 14, 4, 17, 13, 10, 14, 0, 24, -4, 31, 0,
    21, 17, 3, 17, -1, 12, 16, -9, -6, 4, 2, 23, 9, 12, 7, 1, 10, 2, 0, 6, -1, 7, -6, -12,
    7, -8, 16, -5, 8, 3, 3, 9, -2, 2, -14, -5, -5, -4, -1, 1, -6, -13, -7, -12, 14, 9, 12, -8,
    8, 9, 15, -9, -2, -6, 16, -9, 0, -13, 6, -12, 2, 0, -5, 5, -9, 4, -8, -6, -17, -15, -11, -10,
    -17, -19, 8, -18, -10, -18, -14, -9, -11, 2, -17, 8, -31, 5, -24, 13, -17, -6, -19, -1, 1, -6, -12, -8,
    12, -9, 14, -18, 14, -10, -7, -11, -19, -1, -8, -9, -18, -25, -3, -10, 1, -13, -15, -15, -28, -2, -6, -16,
    0, -30, -14, -14, -18, -5, -16, -8, -12, -25, -19, -25, -20, -4, -31, 4, -20, 2, -6, -5, -6, -8, -8, -9,
    -8, -14, -9, -11, -6, -19, -21, -3, -24, 15, -31, 1, -26, -18, -25, -28, -26, -34, -15, -27, -18, -37, -33, -43,
    -32, -37, -24, -32, -28, -44, -40, -49, -34, -43, -40, -52, -37, -41, -39, -23, -30, -9, -36, -16, -30, -16, -19, -26,
    -19, -40, -28, -36, -25, -34, -29, -27, -26, -38, -28, -39, -46, -26, -71, -29, -74, -47, -71, -63, -35, -49, -22, -60,
    -31, -47, -21, -36, -21, -30, -32, -23, -37, -8, -32, -3, -26, -2, -17, -9, 1, -8, 28, -1, -6, -17, 6, 8,
    10, -12, 0, 13, 6, -2, -14, 20, 16, 34, 3, 14, 28, 26, 29, 12, 20, 38, 26, 30, 22, 14, 25, 15,
    37, 15, 29, 18, 27, 24, 39, 26, 36, 42, 33, 43, 15, 53, 45, 50, 51, 38, 61, 35, 50, 47, 41, 45,
    44, 50, 39, 35, 43, 38, 43, 38, 38, 47, 48, 52, 64, 53, 66, 73, 55, 81, 46, 87, 54, 68, 74, 51,
    76, 41, 70, 46, 63, 36, 53, 43, 45, 38, 42, 41, 27, 40, 32, 49, 43, 36, 17, 37, 26, 44, 30, 30,
    29, 18, 39, 17, 39, 21, 29, 27, 35, 11, 6, 2, 8, 17, 12, -3, 8, 4, 7, -15, -10, -14, -13, 8,
    -28, 12, -6, 9, 3, 3, -10, 12, 2, 22, 6, 16, 10, 12, 20, 18, 20, 30, 5, 32, -4, 11, -12, 4,
    0, -10, -7, -13, 0, -5, 4, -20, 7, -39, 8, -43, 5, -30, -16, -26, -32, -4, -31, -6, -29, -14, -16, -18,
    -33, -14, -27, -4, -33, -13, -26, -11, -13, -26, -8, -22, -15, -15, -22, -29, -32, -41, -25, -34, -37, -28, -50, -21,
    -51, -38, -37, -42, -35, -63, -42, -57, -45, -49, -58, -40, -61, -46, -61, -52, -55, -49, -42, -47, -55, -56, -65, -55,
    -57, -54, -60, -61, -62, -63, -53, -71, -56, -62, -50, -55, -57, -66, -53, -47, -51, -38, -65, -28, -62, -32, -53, -36,
    -25, -35, -24, -36, -16, -25, -13, -34, -4, -28, -17, -11, -24, -15, -15, -22, 3, -27, -6, -23, -16, -16, -32, -5,
    -35, -14, -37, -14, -33, -11, -26, -15, -13, -20, 1, -7, 8, 1, 5, -3, 15, -3, 34, 1, 24, 29, 37, 32,
    31, 36, 52, 41, 44, 30, 34, 48, 23, 25, 28, 23, 27, 10, 19, 17, 21, 10, 5, 21, 8, 17, 8, 16,
    23, 19, 10, 22, 14, 41, 21, 28, 17, 9, 19, 7, 24, 14, 28, 19, 24, 9, 25, 8, 28, 13, 15, 21,
    14, 27, 30, 28, 43, 6, 31, 19, 32, 23, 11, 33, 31, 43, 38, 26, 46, 44, 41, 49, 35, 47, 40, 34,
    44, 24, 43, 25, 47, 43, 34, 52, 30, 61, 39, 51, 45, 53, 33, 45, 33, 43, 20, 28, 18, 18, 19, 20,
    13, 21, 12, 16, 23, 21, 30, 18, 32, 3, 33, 13, 30, 8, 11, 26, 12, 23, -10, 6, 17, 3, 6, -6,
    -4, 6, -14, -15, -17, 2, -9, -13, -35, -12, -19, -7, -10, -24, -7, -15, -6, -4, -12, -3, -21, -6, -25, -19,
    -37, -27, -19, -33, -14, -29, -13, -19, -18, -20, -3, -13, 1, -12, 0, -7, 12, -10, -5, -14, 0, -16, -16, -15,
    -11, 11, -5, -9, -4, -6, 1, -18, -4, -16, 0, -30, -8, -24, -2, -17, -17, -21, -21, 6, -26, 6, -24, 6,
    -7, 1, 1, -5, 2, -2, -13, -10, -22, -5, -7, -9, -1, -6, -2, -3, -14, 0, -28, 6, -27, 1, -30, -9,
    0, -11, -16, -40, -12, -35, 1, -38, 4, -26, 13, -9, -5, -1, -15, 2, -36, -9, -44, -21, -27, -36, -17, -55,
    7, -26, 0, -24, -14, -20, -8, -39, -7, -50, -21, -38, -29, -25, -36, -3, -17, -10, 2, -6, 13, 4, 9, 25,
    14, 34, 30, 26, 41, 11, 39, 29, 29, 36, 30, 39, 28, 47, 23, 53, 20, 44, 14, 13, 5, -11, 16, -8,
    1, -11, -21, -32, -30, -46, -46, -56, -70, -69, -66, -53, -54, -53, -53, -60, -47, -68, -52, -60, -40, -41, -16, -22,
    -20, -30, -24, -24, 2, -24, -15, -11, -34, 5, -29, 3, -5, 33, 6, 0, 7, -3, 23, -3, -4, 8, -26, 11,
    -7, -3, 3, -10, 4, 11, -16, 11, -19, 37, 17, 23, 29, 7, 29, 15, 37, 18, 45, 6, 48, 11, 36, 21,
    32, 29, 48, 44, 26, 47, 35, 76, 21, 26, 18, 22, 24, 21, 26, 32, 15, 45, -2, 44, 22, 18, 35, 8,
    47, 26, 46, 51, 54, 35, 41, 31, 59, 46, 55, 54, 51, 59, 69, 69, 59, 70, 51, 77, 56, 69, 45, 62,
    44, 51, 54, 53, 84, 52, 75, 53, 69, 63, 60, 58, 67, 52, 47, 60, 32, 56, 40, 55, 35, 45, 38, 31,
    50, 33, 38, 27, 52, 34, 51, 23, 50, 24, 57, 5, 55, 15, 57, 28, 29, 44, 6, 42, -5, 47, -5, 34,
    5, 13, 8, 8, -7, -7, -6, 0, -8, -9, -19, -2, -23, -7, -30, -15, -25, -16, -16, -29, -22, -36, -24, -31,
    -30, -32, -34, -23, -27, -27, -36, -34, -37, -39, -48, -55, -73, -46, -67, -29, -71, -43, -63, -46, -43, -47, -44, -61,
    -38, -53, -32, -58, -36, -55, -55, -59, -62, -56, -57, -58, -52, -53, -70, -64, -72, -61, -65, -72, -70, -77, -60, -63,
    -59, -64, -62, -50, -59, -51, -52, -58, -55, -60, -66, -48, -68, -55, -66, -52, -66, -56, -51, -46, -35, -56, -30, -29,
    -19, -7, -18, 2, -11, -11, -12, -13, -7, -5, -14, -12, -9, 2, -5, 5, 2, 5, 3, 8, 1, 5, 9, -2,
    16, 7, 24, 5, 18, 26, 25, 28, 15, 42, 24, 40, 45, 44, 55, 43, 30, 68, 34, 49, 31, 47, 45, 42,
    47, 27, 61, 39, 46, 25, 50, 26, 65, 30, 44, 42, 17, 60, 13, 56, 15, 27, 31, 24, 32, 19, 30, 26,
    16, 13, 19, 30, 32, 27, 21, 27, 33, 23, 38, 17, 42, 21, 6, 43, 1, 57, 19, 52, 44, 37, 57, 37,
    69, 33, 62, 41, 66, 54, 67, 48, 68, 45, 51, 49, 37, 60, 25, 53, 21, 46, 34, 10, 24, 7, 27, 14,
    13, 17, 20, 13, 4, 13, 8, 30, -3, 19, -8, 37, -6, 19, -17, 9, -2, -7, -5, -15, -3, -13, -5, -24,
    -8, -21, -8, -29, -15, -31, -25, -36, -39, -25, -54, -6, -57, -10, -43, -26, -36, -38, -23, -41, -19, -40, -23, -28,
    -14, -38, -17, -29, -20, -24, -22, -33, -24, -22, -37, -16, -42, -28, -22, -23, -17, -16, -17, 1, -14, 2, -3, -12,
    -4, -19, -15, -27, -14, -19, -8, -20, -22, -26, -24, -31, -27, -22, -33, -16, -40, -25, -32, -24, -33, -34, -35, -64,
    -32, -49, -31, -30, -34, -19, -21, -20, -9, -29, -14, -49, -10, -50, -10, -30, -18, -22, -18, -3, -11, -7, -9, -9,
    -2, -17, 3, -1, -13, 0, -3, 3, 4, 6, 1, 9, 15, 11, 24, 17, 24, 18, 38, 18, 29, 5, 31, 35,
    24, 25, 12, 34, 16, 21, 24, 18, 27, 23, 26, 16, 19, 19, 23, 12, 21, 11, 14, 11, 5, 17, 17, 6,
    14, 13, 12, 19, -8, 12, -6, 5, -3, -10, 3, -1, -6, -5, -29, -10, -29, -10, -9, -19, -3, -33, -18, -22,
    -27, -16, -25, -14, -34, -23, -15, -24, -5, -31, -22, -14, -20, 0, -10, -4, 5, 5, -9, 2, -6, 6, 3, 15,
    8, 17, 3, 9, -3, 2, 12, 5, 4, 7, -5, 2, -9, 5, -6, -8, 0, -19, 20, -8, 10, 2, 9, 10,
    23, 16, 9, 21, 15, 10, 23, 14, 6, 17, 5, 5, 6, 18, 13, 5, -5, 12, 6, 11, 8, 0, 15, -3,
    19, -18, 7, -14, -10, -11, -17, 8, -11, 23, -6, 6, 14, -10, 26, 0, 8, 7, -2, 9, -8, 1, -4, 8,
    10, 10, -12, 17, 0, 12, 18, 17, 8, 31, 15, 15, 30, 6, 26, 22, 25, 11, 25, 9, 25, 11, 20, 3,
    29, 10, 20, 25, 4, 22, 1, 13, 14, 3, 10, 0, 11, 14, 7, -7, -12, 7, -5, -8, -8, -2, -8, -2,
    -21, -8, -23, -4, -27, -17, -15, -14, -20, -19, -13, -27, -17, -37, -33, -34, -35, -39, -37, -36, -39, -34, -46, -36,
    -45, -29, -51, -34, -50, -43, -54, -48, -45, -39, -44, -40, -43, -48, -38, -42, -45, -40, -32, -25, -31, -21, -30, -22,
    -19, -14, -20, -21, -42, -28, -22, -31, -18, -30, -4, -17, -14, -21, -29, -4, -9, 2, -5, -4, -2, 5, 9, -2,
    8, 6, 11, 12, 12, 10, 6, 31, 13, 21, 8, 12, 25, 11, 4, 4, 7, -2, 24, -17, 6, 7, 1, 22,
    -3, 31, 7, 24, 7, 13, 22, 9, 10, 4, 5, 8, 21, 8, 16, 11, 15, 14, 25, 29, 28, 31, 22, 34,
    6, 33, 11, 28, 18, 17, 11, 17, 37, 9, 33, -2, 30, 13, 18, 12, 5, 36, 8, 27, 6, 20, 35, 9,
    36, 3, 46, 23, 32, 21, 27, 26, 23, 39, 13, 39, 21, 27, 10, 32, 9, 25, 23, 30, 22, 36, 26, 37,
    34, 31, 36, 45, 35, 20, 16, 8, 19, 11, 5, 5, 3, 3, -3, 3, -11, -9, -17, -14, -18, -21, -13, -26,
    -3, -38, -14, -38, -22, -12, -13, -20, -26, -12, -20, 3, -20, -2, -14, -1, -4, -20, 7, 0, 21, 11, 24, 11,
    35, 24, 28, 23, 24, 28, 22, 38, 23, 39, 22, 26, 33, 18, 16, 6, 23, 2, 18, -19, -8, -7, -19, 10,
    -8, -2, -6, 2, 1, -17, -1, -24, 19, -25, -1, -19, -15, -18, -13, -12, -29, 2, -19, -22, -10, -22, -10, -30,
    -20, -35, -17, -40, -18, -38, -29, -51, -29, -39, -33, -39, -26, -27, -26, -34, -42, -17, -37, -9, -37, -4, -19, -8,
    -21, -20, -6, -2, 0, -11, -11, -21, -4, -7, -9, 6, -10, 16, -7, 15, -5, -11, -8, -4, 0, -16, 0, -24,
    6, -17, -12, -12, -4, -9, -3, -4, -17, -7, 2, -2, 10, -2, 2, 8, 16, -1, 7, 7, -10, 18, 1, 11,
    14, -2, 24, 6, 21, -11, 18, -2, 25, 5, 19, 8, 20, 25, 10, 23, 14, 45, 20, 35, 7, 28, 15, 15,
    22, 7, 17, 12, -2, 18, 5, 5, -3, -7, 5, 10, -12, -3, -17, 21, -5, 0, 6, -13, 18, -6, 4, 3,
    -2, 12, -2, 16, 8, -3, 15, -24, 3, -11, 7, 0, 2, -7, 4, 12, -2, 8, 4, 4, 13, 8, 8, 0,
    2, 12, -12, 11, -10, 10, 8, -5, -7, 1, 4, 6, -12, -3, -7, -10, -15, -14, -34, 0, -23, -8, -26, -15,
    -5, -14, -7, -20, -4, 8, -27, -14, -32, -11, -8, -14, -20, -25, -9, -6, 3, -8, -5, -14, -10, -20, -18, -7,
    -13, -4, -24, -20, -26, -13, -12, -8, -11, -1, -22, 0, -28, -11, -20, -18, -2, -3, -4, -10, -3, 2, 0, 0,
    -7, -7, 12, 2, 4, 11, 0, 24, -1, 19, 16, 27, 15, 26, -12, 25, -6, 22, 13, 9, 22, 4, 24, 4,
    16, 14, 16, 13, 5, 6, 7, 17, 0, 16, 1, 26, 13, 25, 17, 33, 18, 18, 1, 13, 16, 10, 15, 6,
    14, 7, 3, -19, 16, -8, 15, -11, -6, -6, 2, -6, -6, -12, -6, -5, -15, -7, -23, -17, -22, -25, -18, -26,
    -5, -25, -14, -39, -17, -23, -15, -21, -3, -20, -4, -23, 2, -13, -17, -16, -29, -2, -26, -15, -44, -20, -13, -17,
    -15, -11, -1, -9, -1, -13, 5, -9, 8, 6, 12, 12, 1, 17, -1, 19, 17, 25, 23, 22, 12, 18, 12, 4,
    21, 7, 27, 10, 30, 14, 31, 14, 27, 23, 33, 13, 30, 23, 23, 31, 11, 24, 9, 39, 18, 23, 6, 9,
    16, 19, 11, 16, 13, 13, 12, -4, -9, -5, 1, -3, -10, -24, -29, -18, -20, -20, -34, -19, -27, -2, -31, -17,
    -24, -10, -11, -21, -16, -20, -9, -20, -14, -23, -22, -19, -20, -5, -22, 7, -10, 14, -9, 7, 2, 11, 12, -3,
    7, -8, 7, 3, 4, 17, 9, 14, 12, 7, 19, 12, 25, 18, 14, 16, 7, 17, 1, 16, 9, -4, 9, 8,
    4, 12, 2, 12, 1, -5, -3, -15, -3, -10, 11, 1, 6, 3, 9, 2, 12, 2, 19, 18, 11, 15, -5, 13,
    -5, 13, -8, 8, -7, -5, -6, -4, -14, -16, -15, -17, -4, -22, -7, -24, -16, -34, -21, -31, -21, -18, -39, -26,
    -38, -10, -28, -24, -36, -42, -21, -36, -22, -19, -23, -13, -11, -6, -6, -6, 12, -9, -6, -10, -9, 0, -15, -19,
    -18, -23, 5, -13, -8, 3, 3, 18, 7, 21, 9, 17, 19, 3, 38, 16, 17, 16, 13, 19, 12, 18, 27, 23,
    39, 16, 39, 36, 52, 36, 43, 31, 63, 30, 52, 21, 46, 43, 29, 27, 19, 52, 27, 42, 26, 46, 45, 43,
    37, 34, 42, 42, 33, 22, 9, 0, 19, 0, 8, -8, -3, 0, -14, -2, -22, 8, -1, 10, -9, -9, -5, -13,
    -6, -3, -11, -10, -19, -26, -8, -25, -24, -22, -25, -5, -20, -17, 5, -2, 18, -16, 7, 8, 16, 31, 19, 29,
    39, 42, 47, 46, 36, 46, 49, 61, 59, 34, 62, 55, 67, 61, 54, 70, 69, 76, 53, 52, 37, 59, 39, 42,
    38, 25, 51, 40, 40, 34, 35, 42, 38, 34, 25, 20, 40, 18, 23, 24, 9, 23, -3, 15, -9, 3, -19, -22,
    -14, -33, -4, -41, -17, -29, -37, -22, -37, -36, -29, -35, -25, -56, -25, -55, -26, -49, -49, -39, -40, -29, -58, -32,
    -45, -42, -45, -38, -46, -31, -49, -58, -30, -68, -36, -74, -35, -52, -50, -51, -54, -37, -49, -40, -51, -39, -42, -32,
    -50, -15, -42, -30, -40, -33, -38, -30, -46, -29, -48, -30, -47, -39, -51, -34, -48, -36, -44, -42, -51, -53, -40, -66,
    -36, -78, -41, -78, -53, -73, -81, -49, -67, -58, -81, -79, -69, -54, -80, -71, -77, -62, -43, -68, -53, -56, -23, -22,
    -45, -34, -43, -7, -23, -18, -18, -9, -15, 2, -7, -17, 0, -8, -9, -6, -16, -12, -7, -8, -3, -9, 2, -4,
    -4, -9, -10, 5, -6, -14, -4, -18, -7, -21, -11, -8, -6, 4, -3, -17, 6, -10, 1, -11, -14, -3, -12, -18,
    -1, -30, -2, -32, -24, -19, -31, 6, -30, 3, -30, 0, -13, -2, -2, -13, 7, -9, 20, 13, 23, 39, 22, 51,
    19, 53, 42, 42, 71, 34, 55, 41, 45, 52, 45, 47, 44, 55, 37, 48, 47, 65, 62, 72, 45, 56, 67, 38,
    55, 47, 41, 56, 31, 34, 43, 43, 47, 52, 45, 47, 49, 25, 38, 18, 42, 26, 28, 10, 8, 33, -2, 10,
    -1, 2, 0, -2, -10, 14, -9, 3, -21, 19, -20, 7, -5, 2, 3, -5, -7, -13, 2, -19, -4, -31, -4, -16,
    -8, -14, -20, -8, -18, -8, -27, -13, -14, -8, -7, -1, -12, 22, 11, 27, 22, 9, 29, 11, 32, 14, 19, 32,
    18, 28, 27, 32, 30, 35, 20, 32, 26, 30, 19, 6, 23, -2, 15, 6, -3, 26, -2, 11, -3, 0, 5, 1,
    9, -11, -1, -16, -2, -11, -9, -8, -19, 6, -19, 7, -23, -6, -12, -26, 2, -26, 14, -17, -4, 2, 10, 13,
    -4, 9, 6, 16, 7, 0, -1, 22, 13, 19, 7, 12, 25, 14, 31, 4, 33, 9, 40, 12, 27, 3, 15, 21,
    -1, 10, -10, -10, 16, -13, 4, -12, 20, 15, 7, 19, -2, 18, 3, 9, -20, 3, -27, 0, -22, -14, -22, -20,
    1, -11, -14, -14, -5, 0, -6, -1, -28, -27, -17, -38, -32, -37, -12, -25, 3, -21, -16, -24, -10, -29, -15, -21,
    -8, -21, -9, -27, -12, -10, -17, -16, -9, -10, -12, -15, -18, 1, -24, 4, -15, 4, 3, -21, -11, -21, -12, -28,
    -47, -30, -32, -18, -28, -23, -51, -27, -43, -12, -29, -13, -10, -8, -17, -24, -23, -5, -7, -1, -12, -19, 0, -10,
    1, 12, 2, 11, 17, 9, 13, -5, 13, -4, 17, -11, -2, 3, 3, 4, 8, -16, 9, 8, 15, 22, -2, 23,
    6, 25, -7, 16, 12, 14, 8, 12, 1, 17, 4, 13, 11, 19, 8, 17, 12, 34, 9, 18, -1, 7, 6, 3,
    0, 2, 4, 4, 5, 14, 4, -2, 15, 7, 20, 5, 1, 1, 12, 16, 7, 15, 1, 10, -11, 2, 3, 0,
    6, -2, 19, -8, 36, 0, 40, 4, 17, -1, 28, 7, 5, 17, 2, 22, 8, 21, 24, 36, 26, 19, 9, 17,
    10, 24, -2, 16, 7, 16, 29, 4, 33, 12, 36, 17, 26, 21, 15, 15, 29, 28, 31, 14, 25, 15, 9, 31,
    11, 24, 16, 13, 17, 19, 7, 22, 11, 17, 17, 11, 14, 0, 5, -15, 5, -26, -15, -27, -13, -22, -23, -26,
    -24, -39, -20, -41, -9, -24, -19, -19, -25, -23, -23, -20, -22, -20, -15, -8, -6, 0, -14, -8, -9, 2, -15, -11,
    -6, 6, 11, -2, -3, -2, 1, -6, 3, -1, -5, 2, -9, 4, 3, -7, -2, -14, -4, -2, -6, -5, -6, -5,
    -21, -6, -13, 4, 1, 2, -1, -20, 20, -13, 28, -5, 23, 2, 24, 4, 17, 27, 32, 35, 6, 20, 4, 36,
    23, 39, 39, 43, 42, 34, 33, 24, 41, 21, 40, 20, 36, 22, 37, 27, 31, 30, 17, 38, 31, 25, 26, 34,
    26, 33, 26, 21, 29, 32, 31, 33, 19, 27, 18, 21, 32, 25, 25, 24, 37, 29, 24, 32, 21, 37, 20, 9,
    7, 18, -11, 6, -9, -4, 2, -8, -12, -17, -2, -13, 3, -7, -13, 1, -1, 1, -8, -12, -6, -9, 0, -11,
    -7, -31, -4, -11, -7, -24, -6, -11, -4, -14, -34, -23, -29, -25, -24, -30, -27, -22, -27, -21, -32, -24, -31, -16,
    -31, -27, -31, -32, -17, -18, -24, -32, -22, -22, -10, -13, -29, -32, -17, -18, -35, -18, -35, 0, -38, -19, -43, -26,
    -30, -31, -15, -21, -13, -24, -17, -16, -27, -23, -30, -17, -25, -12, -30, -19, -36, -19, -38, -29, -27, -19, -12, -21,
    -20, -7, -8, -3, 3, -10, 3, -11, 1, 3, 5, 7, -4, 0, 6, 14, 4, 16, 6, 5, 12, 0, 14, 3,
    13, 7, -2, -3, -10, 13, -8, 2, -13, -6, 2, -5, 7, 0, -2, -6, -9, -7, -10, 3, 3, -6, -1, -8,
    2, -3, 1, -3, -2, 0, 5, -2, -8, -12, 7, -12, -5, -17, -10, -8, -8, -1, -1, 7, -9, -8, -9, 1,
    -1, 8, -2, 12, -7, -1, -7, 8, -5, 5, -9, -10, -1, -23, -23, -7, -13, -12, -4, -42, 8, -30, -16, -42,
    -17, -32, -20, -39, -33, -31, -21, -26, -32, -24, -32, -4, 4, 10, 2, -1, 6, -6, 0, -2, -3, 9, -12, -12,
    -21, 7, -13, 3, -8, -7, -3, -9, -5, -14, 4, -5, 0, -9, -8, 13, 11, 11, 13, 9, 17, 14, 19, 23,
    42, 28, 44, 41, 38, 31, 42, 33, 31, 39, 29, 38, 25, 38, 46, 46, 42, 44, 35, 27, 33, 26, 22, 29,
    15, 18, 15, 12, -1, -2, 5, 2, -1, -4, 4, -8, 0, -6, -8, -4, -4, -1, 7, -8, -19, -2, -12, 2,
    -4, -2, 3, 4, 12, 1, 8, 14, 6, 6, 6, 2, -1, 3, 5, 12, -2, 6, 6, 18, 10, 12, 17, 9,
    26, 19, 23, 14, 12, 9, 18, 13, 25, 7, 25, 17, 20, 21, 16, 24, 32, 32, 22, 23, 10, 28, 20, 14,
    1, 8, 9, 12, -11, 2, 1, -5, 1, 3, -21, -4, -4, -4, 3, -5, -22, -26, -11, -24, -26, -33, -36, -14,
    -18, -19, -23, -15, -24, -20, -21, -13, -18, -14, -27, -13, -19, -33, -39, -34, -17, -14, -13, -8, -6, -10, -6, -14,
    -17, -4, -16, -6, -12, -5, -6, 8, -6, 19, -1, 17, 4, 21, 8, 1, 15, -8, 16, -6, -13, -9, -12, 2,
    -3, -9, -36, -27, -45, -42, -32, -38, -38, -28, -31, -20, -22, -33, -30, -22, -38, -33, -56, -33, -48, -34, -58, -52,
    -43, -49, -37, -51, -37, -20, -20, -28, -30, -23, -25, -13, -25, -25, -12, -9, -1, -9, -9, 5, 14, 12, 20, -1,
    27, 30, 17, 43, 16, 24, 36, 26, 7, 27, 29, 43, 28, 21, 30, 43, 37, 30, 25, 35, 32, 38, 24, 33,
    33, 32, 35, 25, 19, 38, 22, 28, 30, 37, 34, 41, 35, 25, 53, 34, 39, 34, 38, 23, 49, 31, 36, 35,
    24, 40, 37, 44, 24, 38, 33, 37, 24, 32, 24, 33, 15, 36, 11, 27, 5, 6, 16, 11, 29, -5, 5, 6,
    19, 30, 2, 16, 22, 19, 14, 1, -3, 16, -2, 5, -14, 20, 13, 18, -6, -1, -3, 16, -5, -12, -28, -6,
    -17, -18, -1, -33, -11, -24, -17, -13, -6, -8, -13, -18, -11, -23, -17, -30, -30, -27, -29, -17, -57, -39, -39, -37,
    -32, -37, -14, -27, -18, -43, -30, -17, -28, -17, -20, -13, -9, -3, -16, -6, -7, -11, 10, -4, 17, 6, 14, 11,
    9, 9, 20, 22, 10, 20, 11, 19, 5, -1, -6, 1, 13, 18, 6, 13, 16, 18, 13, 6, 21, 6, 19, 0,
    13, -1, -1, 3, -3, -1, -10, -4, -3, 0, -11, -10, -17, 0, -15, 1, -17, 1, -16, -10, -11, -18, -11, -10,
    -5, -5, -1, -6, -10, -8, 19, 8, 8, 0, 5, 9, 2, 17, -2, 16, 6, 7, -3, 9, -6, 23, -7, 9,
    -10, 10, 16, -4, 10, -16, 6, -9, -18, -19, -13, -28, -21, -28, -21, -19, -32, -29, -27, -38, -42, -46, -40, -45,
    -49, -50, -46, -45, -65, -58, -47, -33, -43, -40, -37, -45, -32, -36, -25, -47, -30, -58, -48, -48, -17, -24, -11, -5,
    -4, 6, -1, -2, -2, 2, 11, 8, 17, 10, 13, 13, 21, 11, 11, 32, 17, 20, 16, 26, 26, 51, 33, 29,
    41, 43, 36, 35, 26, 20, 29, 6, 16, 2, 9, 3, 10, 1, 10, 12, 4, 11, -2, 2, -15, 0, -19, 2,
    -14, -10, -31, -18, -40, -34, -41, -27, -18, -20, -25, -33, -34, -24, -33, -31, -32, -38, -25, -38, -33, -45, -40, -59,
    -51, -37, -29, -24, -42, -33, -39, -13, -15, -20, -11, -21, -10, -7, -5, -11, -13, -20, -8, -8, -4, 6, 17, 16,
    10, -17, 7, -1, 9, -6, 4, -1, 6, 19, -5, 22, 30, 37, 29, 39, 18, 40, 31, 36, 52, 47, 51, 31,
    44, 26, 41, 32, 32, 61, 41, 38, 25, 13, 29, 28, 15, 10, 17, 19, 27, 44, 18, 37, 26, 24, 33, 32,
    38, 41, 55, 32, 34, 30, 29, 40, 34, 30, 37, 36, 30, 34, 23, 21, 32, 14, 26, 15, 12, 12, 24, 11,
    11, 14, 11, 32, 13, 26, 38, 28, 25, 20, 27, 22, 35, 31, 25, 54, 43, 45, 43, 30, 42, 32, 48, 33,
    37, 30, 54, 41, 40, 45, 35, 48, 36, 27, 30, 39, 30, 23, 29, 2, 27, 18, 15, 19, 11, 18, 9, 6,
    7, 21, -5, 7, -16, -24, -23, -27, -13, -35, -25, -33, -32, -37, -34, -30, -25, -40, -47, -39, -33, -24, -37, -38,
    -42, -45, -43, -43, -58, -45, -36, -52, -38, -49, -18, -37, -25, -45, -35, -28, -32, -30, -38, -25, -51, -27, -43, -40,
    -39, -44, -47, -51, -47, -50, -38, -47, -46, -53, -48, -44, -37, -60, -44, -54, -37, -39, -56, -36, -53, -28, -51, -27,
    -41, -29, -34, -34, -21, -36, -31, -28, -8, -22, -16, -23, -28, -11, -28, -22, -40, -24, -30, -33, -24, -29, -12, -11,
    -12, -15, -24, -12, -19, -5, -5, -8, -3, -9, 0, 8, 6, 11, 22, 21, 5, 24, 9, 26, 21, 24, 13, 32,
    8, 23, 10, 18, 28, 20, 5, 6, 13, 22, 14, 22, 14, 30, 16, 26, 10, 29, 14, 15, -5, 11, 9, 17,
    17, 25, 5, 18, 14, 10, 28, 15, 29, -4, 36, 8, 28, 29, 8, 17, 25, 26, 24, 19, 13, 32, 26, 31,
    9, 22, 32, 27, 33, 17, 35, 43, 25, 36, 18, 41, 35, 40, 33, 39, 45, 46, 30, 41, 42, 54, 36, 55,
    48, 44, 45, 44, 53, 49, 39, 49, 44, 49, 37, 28, 29, 46, 35, 36, 25, 25, 35, 27, 21, 15, 18, 18,
    20, 16, 24, 6, 25, 10, 19, 16, 9, 17, 5, 25, 9, 15, 4, 14, 4, 20, 4, 19, 8, 5, 10, -2,
    1, -7, 12, -14, -4, -13, -9, -10, -23, -5, -13, -10, -22, -32, -24, -29, -21, -43, -27, -52, -36, -58, -26, -64,
    -31, -42, -17, -34, -27, -37, -23, -18, -23, -34, -16, -36, -29, -22, -45, -12, -25, -35, -43, -42, -41, -46, -20, -56,
    -39, -34, -44, -44, -52, -53, -49, -26, -45, -37, -45, -50, -25, -32, -29, -21, -11, -23, -17, -28, -10, -5, -15, -20,
    -23, -29, -26, -5, -12, -4, -11, -12, -24, -2, -25, -11, -23, -17, 0, -16, 1, -17, 6, -11, 7, -9, 3, 3,
    13, 3, 10, 24, -2, 13, -3, 14, -3, -7, -5, -6, -1, -9, -2, -10, 10, -13, 3, -9, 2, -10, -11, -15,
    -18, -2, -15, -15, -33, -13, -26, 1, -23, 0, -10, 15, 1, 9, 6, 9, 28, 13, 12, 21, 12, 23, 12, 19,
    14, 16, 24, 26, 31, 9, 20, 36, 21, 23, 9, 13, 19, 23, 3, 20, 23, 8, 13, 16, 17, 24, 24, 11,
    24, 31, 16, 27, 21, 34, 29, 31, 20, 19, 23, 21, 32, 12, 24, 30, 27, 14, 16, 17, 15, 22, 17, 9,
    8, 1, 1, 4, -7, 2, -11, 4, 1, -3, -13, -6, -3, -2, 0, -1, -4, -2, -3, -8, -10, -4, -11, -14,
    -12, -19, 0, -23, -7, -16, -1, -23, -20, -41, -19, -3, -20, -13, -23, 4, 1, -4, -6, -17, 8, -10, -12, -34,
    -14, -17, -14, -25, -30, -35, -21, -26, -47, -24, -13, -8, -14, -6, -42, 3, -16, -6, -5, -5, -10, -3, 9, -7,
    19, -9, 14, 13, 25, 4, 30, 1, 27, 24, 20, 16, 6, 14, 18, 23, 13, 14, 18, 12, 31, 31, 9, 35,
    26, 26, 27, 21, 37, 22, 38, 8, 18, 21, 15, 41, 22, 26, 35, 26, 34, 36, 30, 24, 39, 35, 25, 40,
    26, 23, 35, 17, 44, 26, 32, 28, 40, 31, 24, 15, 12, 30, 14, 20, 14, 2, 16, 11, 12, -8, 15, 12,
    34, 25, 30, 9, 33, 14, 31, 35, 31, 24, 34, 39, 42, 43, 20, 37, 16, 42, 23, 36, 26, 29, 32, 6,
    22, 13, 8, 4, 13, -11, 19, -4, -5, -5, -1, -1, 11, 3, 4, 8, -9, -5, -2, -16, -11, -13, -26, -20,
    -25, -36, -27, -26, -41, -30, -35, -36, -28, -24, -56, -28, -48, -41, -43, -43, -40, -42, -46, -54, -55, -50, -54, -46,
    -52, -38, -57, -57, -57, -59, -49, -52, -49, -55, -38, -48, -41, -44, -45, -28, -29, -24, -22, -23, -18, -9, -18, -25,
    -14, -30, -11, -22, -17, -2, -7, -18, -29, -14, -21, -9, -11, -34, -4, -22, -15, -35, -35, -33, -42, -34, -43, -38,
    -49, -41, -50, -31, -62, -47, -47, -55, -45, -45, -51, -52, -52, -58, -62, -64, -66, -50, -41, -42, -37, -46, -35, -30,
    -48, -45, -44, -33, -30, -44, -44, -45, -38, -47, -49, -40, -45, -43, -40, -47, -24, -38, -27, -41, -18, -11, -18, -11,
    -16, -8, -3, 0, -14, 0, 8, 4, 9, -3, 7, 17, 18, 23, 23, 19, 28, 27, 24, 34, 31, 27, 29, 27,
    38, 46, 31, 34, 35, 44, 37, 39, 28, 38, 49, 37, 39, 33, 35, 33, 29, 37, 27, 34, 26, 37, 21, 36,
    36, 31, 42, 52, 47, 60, 44, 39, 49, 46, 39, 40, 45, 44, 31, 28, 29, 53, 38, 40, 40, 27, 38, 35,
    25, 23, 31, 38, 22, 38, 21, 30, 37, 26, 30, 37, 31, 40, 54, 38, 47, 29, 37, 46, 48, 38, 44, 54,
    42, 54, 45, 52, 58, 54, 62, 62, 54, 58, 57, 60, 47, 67, 52, 52, 45, 53, 52, 55, 34, 44, 49, 35,
    51, 28, 45, 26, 37, 22, 36, 15, 23, 13, 15, 20, 14, -4, 16, 8, 5, 13, -10, 2, 7, 2, -12, -10,
    -14, -1, -4, -22, -10, -3, -15, -6, -17, -29, -5, -18, -21, -15, -21, -24, -7, -29, -22, -20, -27, -23, -22, -25,
    -35, -28, -48, -27, -37, -39, -28, -22, -19, -28, -33, -48, -15, -32, -25, -42, -29, -40, -30, -51, -38, -52, -35, -42,
    -38, -32, -49, -46, -44, -34, -37, -42, -45, -47, -44, -50, -39, -49, -58, -61, -76, -41, -44, -40, -45, -22, -36, -36,
    -47, -42, -42, -27, -37, -57, -28, -38, -23, -36, -27, -32, -5, -24, -32, -35, -23, -20, -18, -33, -30, -31, -23, -19,
    -24, -41, -35, -41, -34, -32, -41, -50, -16, -32, -9, -33, -14, -18, 4, -8, 3, -3, 13, 4, 11, 4, 18, 21,
    17, 36, 32, 27, 26, 17, 27, 34, 20, 23, 26, 37, 36, 29, 31, 15, 31, 35, 38, 40, 35, 37, 35, 39,
    41, 46, 45, 35, 50, 35, 41, 43, 52, 47, 49, 43, 61, 51, 55, 50, 60, 65, 67, 44, 48, 53, 53, 56,
    44, 36, 42, 40, 46, 22, 48, 41, 45, 37, 22, 28, 11, 13, 13, 17, 2, -4, 3, 8, 9, -2, -20, -7,
    7, 4, -16, -38, -17, -16, -24, -30, -32, -26, -14, -19, -19, -31, -15, -26, -23, -22, -23, -16, -24, -21, -21, -24,
    -26, -17, -26, -24, -2, -11, -15, 2, -13, -11, -10, -25, -20, -23, -24, -47, -27, -59, -20, -40, -25, -52, -33, -39,
    -27, -13, -33, -41, -21, -22, -25, -31, -18, -20, -8, -14, -22, -3, 2, -10, 5, -2, 14, 10, 16, 13, 15, 25,
    23, 17, 27, 20, 19, 18, 24, 20, 3, 16, 20, 20, 40, 40, 30, 43, 39, 8, 40, 54, 37, 44, 54, 24,
    62, 49, 49, 64, 63, 69, 55, 79, 46, 72, 67, 52, 60, 61, 73, 70, 57, 57, 53, 83, 71, 66, 43, 46,
    34, 57, 59, 63, 27, 52, 42, 55, 52, 47, 52, 38, 55, 55, 55, 48, 41, 43, 51, 58, 47, 41, 51, 39,
    38, 37, 56, 45, 51, 55, 27, 44, 58, 51, 60, 57, 32, 41, 32, 27, 25, 16, 12, 29, 10, -5, 4, -11,
    1, 0, 9, 14, 20, -2, 2, -1, 3, 17, 23, 9, -4, 18, -4, 19, 3, 0, 0, 0, -15, -5, -15, -18,
    -34, -50, -57, -54, -51, -56, -82, -70, -71, -75, -103, -77, -94, -72, -75, -86, -79, -75, -80, -77, -78, -73, -86, -68,
    -68, -84, -82, -84, -79, -75, -90, -83, -97, -86, -92, -85, -85, -87, -100, -94, -95, -72, -90, -81, -92, -79, -69, -85,
    -113, -101, -98, -88, -91, -100, -74, -82, -61, -85, -62, -71, -86, -77, -88, -81, -73, -47, -62, -44, -47, -39, -43, -36,
    -28, -31, -28, -31, -32, -20, -35, -32, -26, -19, -18, -22, -26, -30, -26, -14, -7, -5, -12, -14, -17, -3, -3, -3,
    -3, 1, -7, 4, 18, -3, 12, 17, 7, 17, 14, 19, 26, 27, 19, 38, 40, 26, 28, 36, 45, 40, 33, 33,
    32, 50, 51, 49, 40, 57, 51, 47, 38, 32, 30, 43, 49, 28, 30, 31, 29, 44, 34, 36, 39, 46, 25, 35,
    25, 39, 52, 47, 46, 33, 37, 37, 40, 53, 43, 37, 27, 4, 10, 6, 17, 10, 3, 10, 18, 16, 24, 13,
    30, 26, 31, 26, 27, 32, 32, 33, 21, 38, 32, 30, 32, 19, 20, 29, 17, 15, 17, 10, 24, 24, 19, 18,
    16, 28, 22, 33, 31, 44, 44, 37, 38, 47, 38, 48, 35, 30, 23, 16, 32, 14, 26, 4, 6, 7, 8, 7,
    0, 3, -20, -8, 1, -1, -14, -21, -27, -17, -13, -26, -22, -29, -21, -27, -38, -34, -42, -54, -47, -60, -68, -59,
    -69, -71, -64, -66, -69, -55, -73, -61, -69, -69, -69, -61, -53, -54, -62, -68, -58, -51, -66, -51, -50, -42, -32, -42,
    -33, -38, -40, -28, -33, -29, -40, -26, -21, -17, -13, -13, -18, -15, -7, -18, -2, 4, 8, 7, 9, 16, 14, 17,
    1, 20, 12, 17, 12, 15, 9, 14, 9, 19, 10, 18, 10, 13, 14, 15, 16, 21, 21, 9, 30, 15, 31, 22,
    20, 13, 8, 7, -9, 27, -7, 5, 1, 7, 4, 16, 11, 11, 12, 26, -4, 28, -1, 5, 4, 8, -19, 0,
    3, -2, 21, -7, 0, -3, -2, -7, -10, -2, -10, 3, -13, 4, -14, 1, -1, 9, 6, 19, 0, 8, 20, 5,
    17, 41, 11, 27, 15, 25, 19, 35, 17, 46, 38, 18, 24, 43, 15, 40, 17, 39, 40, 46, 39, 45, 40, 35,
    45, 34, 39, 40, 35, 37, 32, 39, 26, 32, 24, 32, 21, 31, 23, 19, 27, 13, 35, 36, 42, 31, 42, 49,
    50, 50, 46, 44, 53, 37, 40, 23, 46, 35, 50, 36, 34, 17, 40, 18, 27, 19, 15, 16, 25, 21, 23, 31,
    24, 7, 20, 11, 21, 21, -4, 4, 10, 4, 7, 6, -6, -10, -4, -11, 0, 0, -5, 2, -7, -18, -8, -2,
    -14, -14, -18, -6, -8, -7, -2, -1, -9, -9, -14, -12, -4, -12, -13, -2, -23, -3, -18, -11, -12, -1, -3, 2,
    2, -2, 2, -2, -3, 5, -14, -10, -9, -16, 0, -12, -18, -14, -1, -12, -14, -18, -18, -4, -17, -23, -23, -34,
    -24, -26, -32, -23, -24, -25, -22, -32, -36, -20, -22, -4, -25, -8, -13, 0, -11, -14, -9, -16, -11, -12, -17, -4,
    -13, -7, -24, -14, -9, -21, -24, -33, -19, -20, -19, -28, -27, -21, -19, -35, -28, -19, -26, -23, -24, -28, -18, -17,
    -21, -8, -10, -12, -5, 5, -8, 6, -3, -11, 2, -5, -10, -5, -4, -6, 11, 14, 0, 24, -2, 4, -10, -3,
    -2, 7, -5, -10, -7, -15, 4, 1, -13, -10, -13, -5, -7, -6, -2, 1, 11, -6, 2, -5, -2, 1, 3, 8,
    -11, 4, -9, -18, -4, -23, -9, -8, -6, -25, -11, -18, -10, -17, -17, -11, -10, 2, -21, 5, -2, 2, -3, -4,
    15, 10, 20, 9, 9, 16, -2, -14, 6, 8, 21, 12, 8, 4, 12, 16, 6, 25, 11, 18, 33, 16, 16, 7,
    8, 8, 24, 28, 19, 19, 13, 14, 16, 21, 11, 28, 23, 35, 20, 19, 29, 34, 32, 22, 18, 19, 27, 14,
    9, 3, 0, 13, 18, 8, -2, -13, -5, 1, -14, -8, -20, -25, -30, -35, -36, -20, -22, -34, -26, -23, -18, -11,
    -20, -28, -27, -33, -28, -2, -21, -11, -12, -8, -5, -8, -15, 7, 1, -1, -7, 0, -9, 9, -1, -3, -4, 3,
    -1, 6, -2, -6, -3, -6, -1, 7, 10, -14, 4, -3, 12, 16, 13, 5, 14, 23, 19, 25, 20, 20, 23, 16,
    16, 0, 17, 9, 18, 16, 14, 12, 9, 20, 9, 31, 22, 18, 27, 16, 13, 26, 9, 17, 17, 24, 23, 25,
    -2, 12, 11, 6, 7, 4, -17, -10, -13, -23, 5, -19, -17, -11, -22, -16, -30, -9, -12, -9, -15, -38, -16, -41,
    -28, -31, -37, -24, -15, -17, -27, -10, -17, -6, -7, 5, 4, 7, 1, 9, 2, 8, 14, 3, 6, 1, 14, 23,
    20, 16, 20, 15, 25, 14, 4, -3, 20, 17, -4, 7, -12, 6, 1, -10, -9, -2, -8, -11, 6, 13, 16, 26,
    14, 27, 21, 31, 24, 36, 29, 32, 36, 38, 43, 26, 38, 26, 29, 17, 29, 26, 37, 24, 27, 28, 23, 23,
    23, 15, 15, 21, 21, 27, 4, -12, 8, 6, 13, 10, 14, 12, 11, 13, 5, 19, 1, 17, 20, 16, 16, -1,
    0, 3, 16, -5, 8, 8, 2, 17, 3, -2, 3, 3, 4, 21, 3, 10, 9, 3, 9, 3, -9, -5, -9, 0,
    -13, -2, -16, -10, -14, -2, -3, -14, -3, -1, -6, -8, -28, -21, -29, -34, -25, -31, -23, -29, -24, -40, -39, -40,
    -35, -42, -44, -46, -50, -49, -40, -45, -53, -46, -55, -42, -33, -49, -50, -48, -57, -65, -71, -79, -75, -69, -71, -74,
    -65, -76, -68, -69, -56, -64, -63, -54, -67, -66, -65, -68, -49, -45, -56, -53, -46, -41, -40, -51, -55, -36, -38, -24,
    -44, -47, -48, -38, -35, -33, -40, -34, -28, -37, -30, -39, -35, -33, -17, -35, -30, -26, -4, -18, -25, -18, -13, -10,
    -4, -26, -23, -26, -20, -20, -15, -20, -15, -20, -24, -30, -16, -11, -13, -22, -17, -6, -5, 4, -10, -7, 9, 22,
    7, 22, 19, 13, 20, 23, 17, 31, 24, 15, 26, 19, 13, 23, 21, 11, 6, 33, 27, 14, 13, 16, 34, 35,
    33, 10, 30, 44, 22, 21, 29, 22, 30, 33, 22, 46, 50, 54, 54, 51, 57, 56, 52, 67, 55, 59, 79, 58,
    67, 83, 55, 61, 80, 64, 74, 71, 47, 64, 72, 48, 68, 67, 63, 62, 35, 54, 65, 55, 60, 64, 35, 54,
    56, 41, 61, 49, 44, 67, 61, 44, 55, 51, 40, 57, 53, 54, 54, 43, 53, 34, 44, 44, 40, 48, 54, 37,
    32, 48, 33, 35, 39, 31, 46, 45, 39, 28, 28, 19, 15, 16, 16, 26, 14, 23, 23, 29, 12, 22, 6, 19,
    18, 14, 26, 17, 23, 10, 12, 23, 13, 16, 24, 18, 23, 27, 9, 4, 1, -3, -11, -6, -11, 11, 8, 7,
    9, 5, 6, 20, 20, 13, 7, -3, -1, -3, -11, -22, -11, -22, -19, -18, -25, -28, -22, -48, -31, -42, -29, -44,
    -39, -45, -51, -73, -38, -69, -72, -65, -67, -71, -54, -71, -79, -47, -53, -56, -47, -60, -57, -36, -49, -65, -51, -62,
    -50, -52, -66, -57, -59, -49, -47, -44, -42, -36, -41, -31, -22, -31, -18, -26, -31, -23, -25, -27, -24, -24, -26, -34,
    -46, -29, -31, -25, -30, -29, -28, -23, -20, -15, -20, -3, -6, -23, -20, -15, -13, -10, 3, -3, -6, 2, -14, 6,
    -6, -9, -6, 2, -11, -11, -13, -20, -13, -34, -38, -47, -42, -40, -41, -36, -34, -23, -26, -25, -30, -25, -6, -9,
    -10, -5, -17, -21, -20, -6, -34, -12, -23, -6, -19, -17, -25, -21, -11, -27, -24, -22, -38, -29, -47, -20, -24, -37,
    -55, -38, -43, -18, -15, -31, -25, -28, -27, -20, -22, -42, -32, -20, -43, -19, -35, -32, -14, -16, -21, -16, -14, -11,
    -7, -17, -26, 2, -22, -16, -14, -21, -7, 1, -16, 2, -8, -12, -12, -2, -15, 10, 2, 2, 2, -6, -5, 12,
    -6, -7, -3, 12, 9, 16, 11, 22, 28, 33, 28, 39, 30, 28, 31, 21, 17, 30, 26, 22, 39, 31, 39, 48,
    36, 43, 41, 50, 52, 48, 53, 34, 44, 40, 43, 38, 41, 42, 25, 37, 24, 28, 28, 29, 40, 46, 49, 49,
    45, 37, 26, 28, 30, 34, 45, 17, 37, 36, 43, 42, 46, 29, 42, 45, 37, 44, 38, 19, 44, 48, 22, 21,
    9, 25, 36, 20, 12, 11, 15, 13, 13, 12, 16, 19, 19, 23, 14, 26, 27, 19, 34, 23, 27, 31, 27, 18,
    11, 20, 28, 34, 21, 25, 29, 18, 38, 33, 33, 24, 39, 23, 29, 28, 30, 17, 24, 11, 19, 30, 34, 18,
    18, 9, 23, 24, 21, -2, 9, 5, 5, 2, -18, -9, -14, -2, -4, -11, -25, -22, -17, -15, -26, -22, -38, -10,
    -34, -28, -33, -38, -27, -13, -33, -14, -45, -15, -24, -21, -36, -17, -17, -11, -7, -23, -12, -19, -9, -1, -17, 3,
    1, -7, 6, -1, -10, 0, 7, -2, 17, 7, 4, 20, 23, -1, 14, 14, 13, 34, 41, 15, 15, 18, 12, 10,
    26, 13, 29, 34, 27, 41, 21, 30, 12, 31, 27, 45, 35, 36, 32, 23, 32, 21, 26, 32, 30, 35, 23, 58,
    38, 30, 40, 35, 29, 41, 28, 25, 44, 26, 20, 32, 29, 21, 37, 25, 29, 44, 24, 37, 24, 30, 28, 23,
    45, 30, 39, 32, 30, 29, 38, 36, 30, 27, 32, 22, 28, 29, 27, 17, 27, 27, 21, 17, 19, 28, 31, 18,
    16, -2, 13, 15, -2, 0, -8, 6, 15, 15, 6, 5, -1, 7, 8, 0, -3, -6, -19, -17, -16, -14, -10, -15,
    -21, -19, -16, -29, -28, -16, -15, -37, -34, -38, -28, -38, -41, -51, -38, -42, -47, -46, -40, -45, -46, -36, -46, -61,
    -48, -55, -45, -38, -49, -45, -50, -55, -53, -48, -51, -57, -59, -56, -44, -52, -57, -62, -68, -55, -54, -75, -76, -62,
    -65, -63, -64, -63, -68, -52, -70, -66, -62, -69, -64, -58, -77, -64, -78, -64, -61, -63, -72, -54, -63, -63, -38, -53,
    -47, -43, -46, -37, -31, -61, -37, -29, -33, -39, -42, -41, -33, -37, -33, -44, -30, -29, -23, -30, -39, -43, -34, -25,
    -34, -33, -42, -29, -19, -23, -16, -28, -19, -16, -26, -24, -18, -23, -15, -17, -13, -17, 1, -5, -3, -3, 4, 1,
    3, 9, 0, 8, 13, 21, 21, 17, 16, 27, 25, 32, 26, 37, 27, 21, 33, 22, 24, 41, 31, 27, 28, 21,
    20, 14, 18, 10, 20, 16, 18, 6, 7, 8, 25, 22, 24, 26, 30, 28, 26, 12, 35, 24, 38, 15, 13, 20,
    25, 30, 22, 22, 24, 20, 24, 22, 12, 29, 22, 38, 16, 25, 23, 26, 33, 30, 33, 27, 28, 31, 25, 19,
    16, 21, 22, 29, 11, 7, 18, 24, 25, 8, 11, 14, 6, 8, 20, 2, 20, 19, 17, 22, 14, 12, 20, 35,
    13, 19, 26, 14, 26, 18, 13, 19, 31, 9, 12, 21, 14, 24, 32, 15, 24, 29, 20, 26, 30, 33, 32, 30,
    22, 26, 26, 20, 36, 18, 32, 29, 26, 32, 32, 26, 33, 40, 30, 41, 39, 15, 34, 27, 31, 35, 28, 17,
    30, 31, 27, 23, 28, 28, 36, 33, 17, 35, 38, 30, 33, 29, 26, 29, 28, 11, 17, 15, 18, 30, 9, 22,
    15, 13, 16, -10, 2, 6, 0, -5, -12, -5, 0, -4, -10, -9, -3, -6, -5, -11, -6, 2, -14, -13, -30, -21,
    -16, -18, -31, -20, -31, -44, -43, -41, -41, -28, -44, -36, -41, -41, -45, -38, -31, -36, -37, -36, -54, -39, -44, -44,
    -33, -49, -35, -29, -47, -30, -35, -38, -44, -41, -38, -44, -52, -33, -34, -32, -35, -28, -32, -29, -27, -35, -22, -26,
    -19, -28, -31, -33, -27, -19, -20, -20, -18, -15, -15, -20, -31, -11, -12, -18, -23, -28, -19, -14, -27, -21, -12, -13,
    -8, -13, -29, -1, -3, -18, 2, -12, -12, -1, -12, -12, 2, -3, -5, -7, -11, -2, 9, -10, 8, 10, 13, -3,
    6, 8, 15, 7, 2, 16, 2, 9, 8, -3, 5, 12, 14, 16, 6, 1, 2, -1, 4, 6, 12, -2, -5, 2,
    10, -1, 1, -3, 3, 7, 11, -5, 8, 0, -3, 8, -8, -11, 7, 9, -6, 7, 3, 4, 6, -3, -5, 17,
    8, 16, 10, 16, 17, 22, 17, 38, 21, 38, 45, 40, 41, 45, 17, 48, 52, 35, 44, 45, 38, 50, 44, 40,
    42, 33, 26, 40, 29, 29, 33, 41, 27, 33, 31, 29, 31, 27, 28, 24, 28, 27, 27, 25, 9, 11, 14, 11,
    4, 6, 10, -1, 20, -9, -3, 9, 2, 6, 13, -20, 0, -2, -10, -3, -7, -10, 2, 7, 0, 9, 4, 2,
    -14, 1, -6, -4, -4, -10, -16, -3, -7, -13, -7, 2, 11, -4, 0, 2, 6, 3, -1, -9, -3, 4, -3, -19,
    -6, -2, -4, 1, -11, -13, 2, -3, -7, 6, -5, -1, -13, -7, -24, 2, -11, 1, 1, 21, 11, 14, 12, 11,
    15, 30, 15, 23, 27, 32, 31, 18, 18, 27, 31, 29, 27, 16, 27, 42, 25, 30, 32, 19, 32, 31, 24, 37,
    24, 17, 26, 23, 21, 17, 18, 29, 32, 19, 24, 16, 7, 18, 20, 11, 28, 16, 23, 31, 13, 24, 27, 28,
    23, 17, 20, 25, 22, 22, 26, 16, 11, 11, -1, 17, 13, -6, 16, 17, 4, 6, 9, -21, 3, 3, -13, 5,
    2, -10, 18, -2, 0, -1, -8, 0, -6, -17, -18, -7, 1, -3, -19, -20, -12, -16, -26, -18, -21, -8, -19, -30,
    -30, -30, -33, -21, -28, -34, -32, -36, -40, -30, -31, -40, -28, -33, -39, -33, -30, -42, -29, -37, -41, -37, -32, -42,
    -22, -43, -38, -35, -41, -35, -39, -57, -47, -47, -53, -37, -57, -54, -42, -53, -45, -41, -51, -33, -41, -45, -54, -34,
    -32, -30, -34, -50, -39, -38, -32, -32, -38, -17, -20, -17, -35, -33, -27, -14, -21, -18, -24, -17, -11, -16, -7, -6,
    -19, -3, -12, -5, -3, -4, -11, -11, -11, -15, -7, -20, -12, -13, -16, -8, -10, -28, -7, -8, -17, -10, -11, -23,
    0, -25, -16, -9, -21, -22, -12, -19, 4, -10, 2, -1, -9, 7, -2, -4, -4, 0, -7, 0, -3, -1, 9, 5,
    -13, -6, 8, 30, 16, 7, 1, 10, 11, 8, 11, 16, 23, 31, 19, 22, 19, 34, 34, 38, 38, 33, 34, 38,
    41, 37, 28, 33, 35, 19, 29, 20, 40, 35, 29, 18, 27, 16, 21, 16, 21, 17, 21, 17, 25, 19, 25, 12,
    39, 36, 22, 34, 27, 13, 29, 23, 27, 30, 25, 31, 36, 38, 23, 31, 19, 27, 39, 43, 33, 17, 19, 35,
    30, 36, 22, 17, 42, 22, 23, 23, 27, 26, 30, 18, 30, 29, 19, 14, 15, 6, 17, 11, 12, 11, 21, 8,
    11, 16, 21, 15, 14, 17, 21, 19, 15, 12, 10, 11, 5, 8, 9, 2, -2, 10, -6, 3, -9, -2, -9, -9,
    -16, -18, -17, -5, -15, -15, -13, -27, -25, -33, -35, -31, -27, -42, -53, -42, -45, -42, -42, -48, -48, -31, -39, -47,
    -38, -41, -48, -35, -45, -57, -40, -40, -37, -44, -40, -44, -39, -43, -37, -35, -41, -38, -38, -32, -40, -49, -40, -48,
    -45, -34, -49, -40, -36, -44, -42, -40, -44, -33, -39, -44, -50, -43, -50, -54, -56, -63, -43, -50, -51, -35, -54, -45,
    -31, -36, -31, -32, -36, -34, -30, -29, -19, -29, -27, -28, -18, -17, -7, -3, -14, -5, -5, -7, -3, -2, -4, 5,
    -15, -15, 2, -7, 3, 9, -3, 17, 7, 2, 2, 2, 13, 10, 3, -2, 3, 0, 5, -12, 7, 8, 11, 1,
    5, 9, 8, 7, 0, 2, 16, 21, 20, 18, 15, 9, 18, 29, 23, 15, 23, 21, 15, 29, 18, 19, 34, 19,
    22, 46, 35, 33, 19, 16, 35, 38, 14, 17, 9, 24, 23, 13, 7, 6, 5, 12, 3, 1, 20, 13, 12, 1,
    8, 19, 17, 5, 15, 6, 16, 19, 17, 32, 17, 20, 29, 14, 24, 21, 23, 34, 36, 38, 50, 19, 22, 26,
    27, 18, 30, 32, 31, 23, 27, 27, 32, 34, 23, 41, 31, 37, 39, 31, 27, 45, 20, 27, 30, 35, 30, 38,
    22, 39, 31, 25, 29, 32, 20, 36, 21, 16, 17, 22, 20, 19, 19, 10, 15, 26, 21, 16, 6, 13, 3, 13,
    12, -1, 1, 8, 9, -2, -2, -3, -7, 11, -13, -7, 12, -3, -7, -11, -11, -3, -2, -9, 0, 0, 12, -1,
    -2, -3, 3, 14, 11, -4, 6, 0, 8, 7, -1, 2, 0, -11, -3, -8, 5, 5, -16, -6, 8, -10, 7, -3,
    3, 9, 3, -2, 2, -11, 5, 0, 2, -5, 12, 5, 14, 12, 4, 4, 18, 4, 8, 10, 3, 2, 8, -3,
    7, 18, 11, 11, 2, -2, 10, 5, -3, 12, -3, -5, 1, -4, -4, 16, -8, -7, 3, -11, 0, -6, -5, 5,
    4, -3, -13, 1, 3, 13, -2, -4, -1, 5, 1, -8, -9, -20, -2, -19, -15, -16, -4, -18, -9, -20, -10, -16,
    -6, -18, -9, -17, -12, -25, -23, -30, -20, -22, -15, -24, -30, -28, -25, -29, -22, -30, -15, -12, -32, -30, -11, -33,
    -23, -14, -38, -26, -20, -26, -17, -30, -37, -10, -27, -11, -21, -25, -21, -25, -22, -18, -21, -31, -23, -35, -24, -30,
    -26, -26, -33, -30, -18, -17, -9, -15, -22, -11, -10, -12, 0, -19, -4, -12, -19, -11, -12, -19, -15, -9, -9, -5,
    -2, 1, -6, -4, -3, 4, -2, 5, -10, 8, 3, 9, 10, 3, -6, 5, 3, -7, 6, 7, 4, 23, 9, 15,
    17, 13, 17, 12, 8, 15, 21, 11, 20, 5, 0, 3, 5, 15, 30, 0, 17, 26, 16, 9, 20, 2, 14, 15,
    10, 23, 24, -4, 0, 13, -3, 18, 22, 16, 22, 8, -6, 19, 18, 13, 11, 13, 16, 38, 8, 23, 15, 20,
    15, 5, 2, 10, 4, 17, 16, 15, 2, -15, 10, 15, 2, -2, -8, -10, -2, -11, -10, -4, -10, -8, -6, -11,
    -5, -8, 10, 7, 2, 6, 3, 11, 17, 0, 7, 22, 12, 17, 19, 8, 14, 22, -4, 25, 20, 19, 26, 20,
    14, 30, 10, 20, 16, 18, 15, 23, 24, 13, 13, 19, 13, 19, 25, 13, 21, 15, 19, 23, 22, 9, 15, 5,
    11, 9, 6, 2, 1, 12, 5, -12, 2, 2, -5, -3, -17, -21, -2, -10, -12, -9, 8, 6, 1, -2, -2, 6,
    10, 5, -2, 2, -9, -9, -11, -8, -13, 2, -2, -1, -4, 6, -10, -3, 4, -6, -5, 9, -12, 11, -3, -17,
    -14, 0, 3, 4, 11, 0, 9, 23, 2, -6, -4, -11, -6, 14, 3, -10, 15, 5, 4, 13, 6, 17, 14, 7,
    13, 23, 0, 4, -16, 18, 22, 23, 10, 11, 3, 22, -6, -9, -11, 7, -13, -6, -20, -21, -20, -29, -34, -29,
    -7, -7, -14, -6, -22, -12, -28, -22, -21, -19, -12, -13, -9, -12, -16, -8, -14, -6, 4, -3, 8, -9, 5, 1,
    -3, 2, -10, 9, -1, 8, 2, 14, -2, 20, 5, 13, 6, 12, 4, 15, 11, 23, 10, 10, 5, 14, -13, 5,
    7, 9, -10, 12, 4, 9, 9, 18, -4, 17, 7, 11, 4, -4, 13, 6, 14, 6, -3, 5, 3, -1, 12, 3,
    1, 5, -10, -13, -3, -1, 10, 2, -4, -8, -5, -13, -9, -14, -15, -9, -24, -20, -23, -22, -15, -28, -18, -16,
    -7, -27, -23, -27, -18, -28, -34, -49, -35, -33, -35, -51, -42, -46, -29, -29, -47, -39, -41, -38, -40, -35, -45, -35,
    -50, -31, -41, -33, -37, -31, -25, -36, -39, -28, -38, -20, -35, -39, -49, -20, -40, -38, -49, -45, -26, -23, -30, -22,
    -29, -21, -17, -17, -13, -12, -21, -34, -6, -13, -14, -7, -23, -9, -9, -10, -7, -3, 7, -1, 2, -1, 20, 20,
    10, 20, 18, 16, 7, 22, 22, 24, 19, 18, 21, 26, 24, 25, 18, 30, 22, 11, 23, 19, 23, 24, 27, 33,
    33, 36, 20, 18, 24, 32, 38, 29, 30, 32, 26, 30, 24, 29, 29, 32, 20, 26, 23, 26, 32, 26, 20, 26,
    17, 18, 23, 9, 21, 15, 19, 16, 29, 11, 23, 14, 10, 6, 15, 8, 26, 5, 19, 29, 20, 15, 16, 11,
    21, 30, 11, 20, 17, 14, 15, 7, -1, 18, 22, 16, -1, 12, 10, 9, 19, 11, -3, 8, 20, 14, 6, 4,
    6, 18, 3, -3, 10, 12, 3, 4, -7, -6, 3, -1, -10, 6, 6, 1, 10, 4, -2, -3, -15, 2, 11, 9,
    6, -10, 1, 1, -7, -5, 5, 1, 2, 17, -10, 8, 2, 11, 8, 3, 0, 6, 21, 15, 1, 2, -1, 3,
    0, 3, -4, 12, 7, 1, 6, 10, 12, 11, 7, -8, 8, -5, 3, 6, 8, -6, 4, 7, 13, 5, -2, 12,
    -3, 11, 5, -2, 3, 19, 8, -3, -19, -9, 11, -4, -10, -7, -17, -5, -14, -14, -25, -13, -23, -21, -18, -18,
    -19, -10, -14, -5, -9, 2, 0, -16, -6, -14, 10, -1, 9, 23, 27, 25, 36, 16, 23, 13, 25, 9, 22, 16,
    15, 20, 8, -10, -4, 4, 16, 12, 8, -3, 9, 6, 7, 15, 8, 18, 5, 13, 8, 18, 31, 14, 19, 23,
    30, 40, 42, 29, 21, 18, 34, 37, 23, 18, 15, 21, 17, 20, 10, 7, 8, 17, 14, 14, 5, 12, 15, 12,
    7, 2, 19, 17, 16, 0, 0, 10, 7, 0, -15, -5, -6, 1, -15, -33, -20, -7, -4, -24, -16, -31, -17, -17,
    -24, -28, -18, -9, 0, -12, -13, -21, -7, -29, -24, -35, -31, -11, -23, -36, -36, -39, -24, -12, -32, -26, -31, -17,
    -17, -6, -30, -14, -13, 1, -11, -17, -13, -10, -12, -5, -14, -18, -12, -14, -6, -8, -20, -16, -26, -21, -33, -24,
    -31, -20, -32, -33, -34, -31, -23, -13, -10, -13, -18, -26, -20, -18, -12, -20, -30, -12, -12, -30, -25, -29, -27, -9,
    -18, -28, -30, -29, -32, -10, -29, -29, -41, -35, -31, -33, -32, -41, -24, -17, -26, -19, -25, -16, -2, -9, -13, -19,
    -16, -10, -10, -21, -28, -31, -21, -11, -22, -36, -15, -16, 4, -28, -18, -21, -6, -6, -9, -20, -4, 6, 18, -6,
    0, -1, 1, 22, -7, -15, 0, 11, 11, 3, -8, -14, 9, -3, 3, -8, -18, 0, 4, -3, -11, -6, 10, 21,
    12, 10, 14, 25, 12, 7, 6, 32, 28, 36, 13, 3, 19, 11, 21, 16, 13, 13, 28, 10, 24, 17, 23, 22,
    21, 18, 24, 28, 23, 26, 18, 46, 38, 37, 41, 30, 26, 39, 21, 29, 39, 27, 31, 40, 24, 34, 37, 33,
    39, 38, 38, 38, 39, 43, 40, 30, 40, 42, 26, 46, 33, 38, 46, 47, 28, 38, 32, 33, 38, 33, 21, 7,
    2, 9, 22, 9, 10, 17, 17, 5, 8, 10, 24, 5, 6, 12, 9, 5, 6, 10, 1, -2, -20, 4, 10, 10,
    -6, 5, 8, 7, 9, -1, -10, -5, -2, -3, -12, 1, -8, 1, -2, -20, -13, 3, 8, -7, -9, -6, -14, -14,
    -17, -21, -15, -25, -32, -28, -23, -34, -16, -18, -14, -33, -35, -45, -21, -35, -35, -35, -34, -26, -30, -30, -34, -29,
    -39, -36, -44, -36, -42, -32, -40, -34, -37, -35, -48, -41, -33, -44, -30, -49, -45, -39, -36, -35, -46, -43, -39, -43,
    -35, -43, -40, -49, -39, -43, -50, -48, -46, -36, -41, -55, -76, -61, -51, -56, -47, -60, -46, -39, -43, -49, -56, -47,
    -41, -34, -42, -37, -31, -28, -29, -51, -46, -28, -29, -32, -36, -47, -33, -33, -37, -36, -38, -33, -33, -29, -32, -23,
    -31, -21, -27, -33, -36, -33, -42, -39, -34, -38, -40, -34, -38, -40, -42, -31, -31, -31, -21, -23, -28, -10, -21, -19,
    -19, -26, -13, -14, -13, -14, -10, -4, -8, -6, -11, -4, -11, 7, -6, -9, -23, -22, -6, -7, -7, 1, -5, 2,
    -8, -10, -11, 7, 11, 23, 30, 7, 1, 18, 16, 25, 26, 1, 16, 30, 11, 12, -6, 10, 9, 4, 17, 15,
    17, 25, 18, 10, 16, 26, 23, 36, 32, 13, 31, 31, 30, 46, 34, 25, 44, 48, 56, 46, 41, 38, 62, 48,
    53, 47, 47, 42, 43, 43, 47, 28, 41, 45, 76, 60, 51, 65, 64, 69, 66, 63, 69, 64, 67, 48, 48, 55,
    49, 55, 55, 38, 46, 54, 48, 51, 55, 39, 46, 50, 35, 38, 38, 45, 42, 41, 41, 35, 52, 58, 50, 24,
    38, 43, 47, 37, 33, 50, 56, 47, 30, 31, 28, 40, 33, 18, 31, 38, 30, 35, 19, 21, 24, 16, 15, 20,
    19, 15, 14, 1, 3, 10, 7, 1, -1, -2, 12, 11, 9, -4, -19, -6, -26, -23, -21, -10, -17, -9, -20, -17,
    -15, -10, -12, -5, -15, -7, -8, -10, -21, -6, -14, -6, -11, -10, -18, -23, -8, -20, -7, -21, -27, -22, -19, -22,
    -24, -22, -35, -24, -31, -36, -24, -37, -40, -34, -38, -30, -55, -29, -30, -23, -27, -34, -16, -15, -35, -46, -39, -26,
    -33, -50, -45, -27, -37, -50, -41, -31, -23, -24, -19, -16, -11, -24, -26, -30, -16, -17, -5, -14, -28, -18, -10, -6,
    -7, -6, -6, 1, 6, -3, 2, -2, -12, -8, -13, -15, -19, -11, -13, -15, -15, 0, -1, 0, 7, 6, 9, 16,
    0, -2, 4, 4, -3, -3, 5, 16, 18, 0, 7, 0, 3, 21, 14, 24, 24, 17, 8, 27, 30, 24, 24, 20,
    31, 40, 39, 34, 37, 43, 58, 45, 45, 32, 51, 63, 50, 36, 13, 26, 28, 32, 18, 16, 24, 27, 32, 26,
    23, 44, 51, 39, 39, 35, 36, 43, 52, 43, 49, 37, 34, 50, 51, 40, 35, 35, 32, 42, 23, 25, 26, 20,
    5, 7, -2, 9, 7, 7, 1, 5, 14, 10, 8, 10, 13, 13, 15, 18, 19, 11, 23, 13, 7, 10, 5, 5,
    17, 1, 6, -3, 5, -4, -5, 8, -3, 10, 2, -7, -5, -3, 8, 12, 3, -3, 6, 22, 19, 18, -9, 11,
    7, 11, -11, 6, -3, 7, 3, -13, -18, 6, -16, 4, -11, -18, -6, -13, -29, -15, -22, -13, -11, -19, -17, -7,
    -12, -3, -9, -16, -1, -5, -15, -7, -8, -7, 0, -3, 4, 11, 5, 17, 17, 6, 15, 10, 15, 11, 9, 4,
    14, 2, 6, 6, 13, 19, 1, 19, 7, 17, 8, 21, -4, 14, -6, 1, -3, 1, -10, -2, -6, -5, -6, 1,
    -9, 10, 1, 20, -2, -8, -18, -1, 0, -13, -26, -27, -31, -15, -16, -16, -12, -12, -25, -10, -22, -30, -13, -22,
    -12, -17, -20, -30, -15, -39, -31, -40, -19, -23, -12, -31, -27, -31, -27, -36, -32, -34, -25, -35, -38, -42, -30, -31,
    -38, -34, -34, -32, -29, -28, -32, -30, -24, -28, -34, -35, -34, -24, -32, -34, -46, -24, -26, -28, -23, -30, -32, -20,
    -23, -17, -31, -27, -25, -18, -28, -32, -26, -27, -16, -26, -39, -37, -29, -31, -25, -34, -25, -36, -26, -31, -21, -23,
    -35, -30, -25, -31, -32, -29, -22, -14, -7, -26, -27, -29, -12, -7, -5, -13, -9, -14, -4, -20, -13, -13, -20, -7,
    -12, -5, -7, -4, -8, -12, -11, -9, -1, 18, -3, 2, 1, -4, -2, 17, 17, 18, 27, 21, 12, 34, 27, 40,
    37, 50, 43, 48, 40, 34, 51, 48, 49, 45, 48, 55, 52, 52, 56, 55, 58, 43, 55, 50, 54, 46, 51, 44,
    37, 33, 32, 31, 41, 36, 31, 25, 36, 43, 42, 39, 47, 44, 54, 45, 43, 51, 55, 47, 37, 24, 22, 28,
    17, -1, -2, 15, 15, 28, -1, 2, 10, 12, -2, 5, -6, 5, -14, -1, -24, -19, -23, -15, -11, -3, -3, 2,
    0, 0, 4, 2, -1, 10, -5, -2, -9, -4, -8, -1, -3, 5, 3, 8, 19, 12, 26, 25, 26, 34, 27, 28,
    28, 21, 23, 25, 15, 22, 20, 20, 26, 29, 32, 37, 31, 24, 20, 16, 16, 17, 11, 8, 7, 2, 1, 6,
    -4, 1, 3, 3, 7, 11, -7, 17, 11, 8, 7, -4, -3, 17, 1, -5, 2, 4, 0, 13, -4, 6, 9, 1,
    6, 6, -10, 4, 4, -11, -9, -24, -19, -11, -18, -36, -30, -18, -16, -16, -28, -27, -24, -21, -36, -42, -39, -26,
    -31, -41, -33, -41, -25, -33, -36, -27, -18, -35, -30, -31, -32, -15, -36, -40, -41, -32, -41, -35, -45, -31, -24, -37,
    -36, -29, -35, -24, -12, -43, -11, -40, -34, -18, -28, -35, -13, -38, -16, -24, -27, -20, -13, -22, -14, -12, -21, -26,
    -18, -23, 0, -22, -20, -16, -14, -21, -1, -16, 8, 10, 4, -3, 8, 11, 10, 21, 5, 18, 12, 4, 7, 26,
    12, 19, 19, 20, 20, 39, 20, 22, 30, 26, 33, 15, 12, 26, 44, 38, 25, 7, 20, 28, 35, 21, 27, 22,
    30, 33, 18, 20, 33, 29, 34, 24, 16, 23, 39, 17, 36, 10, 31, 34, 33, 23, 32, 28, 41, 28, 26, 30,
    32, 26, 31, 17, 20, 11, 9, 7, 13, 5, 20, 8, 16, 12, 23, 13, 28, 14, 16, 17, 18, 13, 17, 8,
    12, 16, 14, 21, 18, 20, 21, 23, 27, 32, 23, 28, 28, 30, 42, 30, 18, 19, 26, 25, 30, 13, 19, 21,
    9, 17, 7, 10, 20, 21, 6, 1, -4, -1, 12, 1, -10, 0, -3, 1, 4, -13, -7, 2, 8, 2, 7, -11,
    5, 1, 8, -3, 1, -2, 3, -5, -7, -12, 2, -4, -6, -10, 0, 7, 3, 1, -2, -18, 4, -9, -11, -17,
    -10, -9, -2, -5, -9, -11, 0, -9, 0, 0, -2, -1, -11, -12, -13, -25, -7, -11, -23, -15, -34, -21, -25, -32,
    -27, -34, -30, -35, -39, -35, -32, -37, -42, -36, -37, -47, -48, -41, -48, -41, -43, -44, -36, -37, -41, -31, -47, -23,
    -45, -36, -34, -46, -39, -46, -47, -42, -57, -49, -44, -43, -35, -52, -50, -38, -44, -51, -50, -40, -46, -40, -50, -44,
    -36, -23, -35, -39, -38, -33, -33, -30, -42, -29, -25, -24, -24, -25, -28, -18, -25, -14, -15, -18, -8, -8, -11, -3,
    -14, -4, -7, -8, -14, -4, -3, -4, -8, -14, 2, 12, 9, 8, 13, 12, 10, 18, 4, 20, 19, 6, 12, 19,
    34, 38, 19, 33, 26, 24, 34, 35, 28, 18, 40, 10, 26, 8, 16, 18, 17, 7, 31, 13, 27, 38, 14, 27,
    23, 18, 19, 19, 10, 12, 2, 22, 27, 16, 6, 9, 16, 20, 21, 11, 11, 18, 8, 13, 12, 2, 19, -4,
    -10, 4, 2, 4, -12, -16, -10, 12, -11, -12, -6, -8, 4, 5, -6, 3, 2, -7, 9, -15, -6, -10, 3, 3,
    -3, -19, 0, 4, -5, -1, 2, 8, 11, 5, 0, 32, 22, 27, 23, 12, 29, 24, 2, 29, 14, 1, 13, 3,
    -4, 21, 3, 8, 1, -5, -5, 4, 4, 0, 4, -10, 0, -5, -22, -17, -20, -6, -3, -15, -18, -15, -15, -7,
    -14, -21, -9, -13, -29, -22, -9, -5, -21, -24, -59, -20, -6, -15, -17, -33, -35, -27, -22, -42, -18, -17, -28, -32,
    -29, -39, -20, -25, -22, -18, -16, -9, -18, -5, -17, -20, -31, -5, -14, 17, 4, -2, 1, 12, 8, 7, -7, -10,
    8, 7, -9, 3, 12, 18, 26, 5, -1, 18, 30, 14, 32, 15, 15, 46, 30, 25, 17, 20, 31, 53, 26, 31,
    40, 43, 58, 46, 39, 35, 44, 58, 45, 38, 44, 40, 48, 38, 20, 35, 41, 38, 53, 34, 29, 48, 35, 34,
    44, 27, 42, 34, 19, 30, 19, 41, 29, 21, 22, 32, 27, 23, 29, 16, 32, 38, 30, 33, 33, 17, 29, 33,
    30, 31, 16, 10, 27, 34, 25, 22, 26, 16, 13, 22, -2, -1, 3, -16, 5, -5, -1, -6, -5, -24, -19, -22,
    -33, -40, -24, -28, -22, -39, -29, -39, -22, -20, -28, -30, -36, -39, -31, -19, -18, -47, -35, -40, -29, -30, -47, -31,
    -26, -30, -47, -35, -24, -19, -21, -39, -37, -31, -44, -43, -23, -28, -19, -42, -23, -39, -34, -39, -17, -19, -13, -17,
    -35, -16, -1, -10, -16, -14, -21, 2, -11, 2, 10, 16, 18, -4, -15, -18, -11, 3, 6, -11, -9, -6, 5, -15,
    -14, -22, -18, -7, -14, -22, -7, -24, -21, -25, -32, -34, -29, -15, -22, -13, -29, -27, -41, -29, -36, -23, -23, -39,
    -36, -18, -33, -50, -32, -30, -28, -8, -32, -44, -17, -9, -31, -21, -36, -26, -6, 5, -28, -15, -17, 5, -5, -6,
    -24, -4, 2, 1, -11, -7, -5, -2, 14, -4, 9, 20, 17, 10, 14, -4, -6, 26, 2, 16, 24, 6, 22, 7,
    11, 29, 26, 35, 35, 36, 21, 29, 36, 48, 34, 22, 44, 29, 43, 36, 19, 41, 49, 55, 44, 59, 52, 73,
    46, 58, 43, 72, 62, 61, 61, 63, 76, 76, 59, 52, 61, 58, 71, 73, 53, 72, 65, 44, 21, 34, 26, 43,
    31, 19, 47, 39, 24, 20, 20, 10, 33, 7, 10, 13, 19, 5, 6, 15, 20, 11, 11, 7, 17, 20, 16, 7,
    10, -6, 5, -8, -6, -17, -2, 10, 10, 0, 6, -10, 5, 6, -19, -10, -18, -10, -13, -14, -29, 5, -16, -12,
    -21, -4, -7, 6, -11, -5, -3, -10, -1, -16, -8, -19, -10, -10, 1, 4, -4, 2, 9, 4, 7, 0, -8, -3,
    0, -4, -10, -8, -17, -3, -2, -2, 1, -4, 4, 4, 1, -2, 7, -9, -7, -16, -19, -7, -11, -14, -24, -18,
    -18, -11, -22, -5, -13, -21, -20, -21, -25, -14, -15, -17, -25, -6, -18, -12, -20, -27, -14, -20, -21, -20, -25, -10,
    -39, -33, -42, -39, -45, -37, -52, -35, -30, -33, -38, -33, -39, -29, -38, -40, -39, -38, -27, -39, -48, -70, -59, -34,
    -42, -53, -54, -49, -46, -40, -63, -40, -29, -35, -48, -49, -69, -42, -65, -58, -50, -38, -37, -32, -46, -28, -33, -31,
    -22, -17, -22, -15, -15, -6, 1, -21, -24, -20, -16, -7, -10, -13, -20, -27, -18, -3, 4, 6, 9, 3, 13, 7,
    -5, 28, 14, 30, 23, 13, 13, 24, 27, 25, 15, 16, 22, 30, 20, 11, 20, 17, 13, 0, 17, 17, 18, 19,
    8, 14, 24, 17, 24, 21, 20, 34, 14, 15, -4, 15, 4, 19, 6, 18, 0, 26, 4, 12, 2, -8, -17, 11,
    -4, 2, 1, 0, -5, 3, -23, -10, 1, 6, 10, 2, -19, -6, -9, 4, 0, 7, 3, 14, -2, 19, 10, 13,
    5, -1, -2, 10, 5, 12, 9, -2, -4, 5, 3, 0, 3, -9, 10, 5, 0, 7, 7, 3, 21, -4, 10, 3,
    16, -5, 17, -8, 4, -5, 3, -4, -11, -5, 23, 12, 11, -5, -1, 17, 8, -8, 4, 8, 11, 4, 11, 1,
    25, 4, -5, 1, 8, 9, 7, -11, 6, 2, 6, 0, 3, -11, -3, -7, -3, -3, -8, -7, -6, -5, -3, -9,
    -15, 0, -17, -16, 6, -6, 4, -16, -14, -9, 12, -3, -4, 13, 7, 5, 8, -1, 1, 15, 7, -2, 1, 1,
    8, 10, 10, 5, 17, 7, 6, 9, 9, 12, 18, 9, 8, 7, 4, 20, 22, 23, 21, 23, 4, 8, 3, 12,
    15, 3, 10, 2, 12, 5, 8, -1, 12, 7, 17, 7, 5, 12, 26, 11, 14, 12, -2, 10, 2, 17, 11, 9,
    -14, 3, -7, 2, 0, -6, -5, 0, 0, 3, -4, 0, 0, 0, 12, 5, -8, -8, -8, -2, -11, -8, -2, 2,
    -4, -11, -3, -3, -10, -2, -2, -13, 3, -23, -12, -25, -15, -14, -15, -23, -18, -32, -14, -13, -27, -16, -20, -22,
    -21, -30, -29, -20, -6, -19, -24, -24, -13, -10, -6, -30, -16, -17, -7, -12, -9, -23, -21, -12, -12, -5, -25, -27,
    -39, -41, -31, -20, -35, -32, -25, -8, -14, 4, -27, -38, -43, -45, -33, -27, -37, -26, -22, -47, -28, -43, -26, -15,
    -16, -19, -12, -3, -3, -12, -14, 1, 5, 11, 7, 3, -1, 22, 17, 21, 25, 19, 5, 26, 11, 23, 26, 7,
    29, 31, 36, 32, 31, 29, 37, 39, 58, 50, 57, 62, 57, 66, 59, 60, 69, 89, 86, 76, 91, 73, 78, 61,
    62, 75, 92, 103, 86, 75, 68, 62, 98, 75, 71, 68, 68, 79, 91, 75, 80, 88, 95, 92, 88, 88, 91, 93,
    99, 97, 101, 83, 75, 62, 75, 86, 84, 69, 64, 76, 77, 77, 63, 61, 67, 70, 58, 62, 65, 59, 56, 51,
    40, 58, 62, 61, 63, 47, 51, 49, 51, 37, 36, 36, 21, 22, 15, 20, 22, 18, 9, 20, 7, 17, -1, -5,
    -5, 5, 0, 2, -11, -21, -17, -3, -10, -12, -31, -35, -21, -17, -29, -33, -34, -32, -39, -43, -44, -33, -36, -41,
    -48, -52, -47, -45, -54, -53, -48, -44, -51, -52, -56, -59, -56, -55, -52, -50, -54, -57, -59, -53, -55, -69, -58, -62,
    -51, -43, -49, -61, -57, -59, -70, -59, -56, -67, -46, -67, -53, -53, -50, -60, -40, -44, -35, -32, -44, -66, -65, -71,
    -52, -35, -39, -45, -49, -43, -39, -41, -67, -57, -60, -48, -58, -70, -63, -54, -60, -63, -63, -60, -46, -40, -50, -61,
    -47, -39, -38, -37, -57, -36, -31, -34, -40, -40, -42, -27, -27, -35, -27, -21, -29, -22, -20, -20, -16, -29, -34, -22,
    -26, -14, -16, -17, -23, -8, -24, -12, -6, -6, 2, 2, -16, -6, -10, -5, -4, -2, -6, -11, -1, -7, 7, -1,
    7, 8, 8, 3, 4, 5, 16, 5, 8, 7, 16, 2, 13, 5, 0, 3, -6, -15, 0, -6, -1, -1, -1, -7,
    -10, -15, -4, -12, -1, -6, 0, 2, -3, 3, 7, 11, 9, 8, 1, 10, 19, 26, 19, 12, 4, -1, 10, 1,
    19, 7, 17, 7, 5, 5, 5, 3, 10, 12, 16, 24, 20, 12, 5, 5, 16, 16, 13, 12, 3, 16, 15, 5,
    7, 7, 26, 17, 12, 4, 11, 2, 0, -8, -8, 2, 2, 12, 10, 9, 16, 8, 18, 17, 16, 19, 25, 12,
    23, 9, 1, 2, 9, 15, 24, 20, 13, 18, 25, 19, 24, 28, 43, 43, 29, 22, 14, 16, 22, 30, 28, 32,
    28, 30, 37, 29, 35, 27, 33, 30, 27, 21, 37, 44, 40, 32, 25, 26, 35, 35, 38, 41, 39, 38, 37, 36,
    27, 20, 25, 31, 30, 32, 12, 15, 33, 33, 22, 23, 17, 13, 23, 25, 18, 36, 17, 21, 17, 23, 16, 19,
    13, 19, 19, 20, 11, 8, 16, 16, 16, 14, 14, 13, 2, 6, 3, 7, 3, -2, 5, -2, 6, -4, 6, -2,
    7, -18, -2, -2, 1, 3, 3, -8, -13, -10, -10, 9, -8, -11, -21, -10, -18, -20, -15, -10, -4, -19, -16, -23,
    -14, -11, -19, -28, -35, -35, -28, -40, -42, -35, -37, -34, -37, -37, -31, -22, -33, -38, -39, -38, -33, -40, -43, -40,
    -39, -38, -35, -52, -30, -36, -32, -28, -33, -25, -22, -30, -42, -46, -46, -32, -23, -34, -33, -25, -21, -24, -25, -36,
    -30, -29, -24, -21, -15, -13, -15, -15, -19, -14, -14, -14, -18, -1, -8, -4, -5, -7, 6, -3, -5, -12, -3, 0,
    5, -4, 4, -1, -1, 0, 2, 6, 0, 2, 3, 11, 24, 18, 8, 15, 2, 1, 9, 4, 9, 7, 4, -4,
    12, 0, 3, -3, 1, 3, 9, 8, 1, 10, 9, 3, 7, 13, 13, 16, -2, 0, 17, 14, 11, 7, -2, 5,
    19, 6, 7, 14, 2, -4, 8, 2, 9, 2, 4, 1, 15, 12, 9, 19, 15, 17, 2, 7, 20, 23, 26, 0,
    11, 6, 27, 13, 17, 9, 21, 27, 35, 15, 13, 23, 23, 21, 0, -3, 2, 21, 20, 20, 13, 24, 13, -7,
    -14, -5, 19, 28, 28, 14, 21, 18, 11, 9, 8, 19, 19, 21, 12, 18, 13, 3, 3, 3, 6, 3, 12, 10,
    27, 17, 1, 6, 7, 15, 11, 14, 1, 4, 0, -10, 5, 6, 6, 17, 24, 21, 8, -14, -16, -19, -8, -6,
    -16, -17, -10, -15, -15, -13, -16, -9, -13, -2, 2, 5, -1, -2, 3, -1, -13, -23, -19, -17, -3, -4, -20, -10,
    4, 9, 4, -8, -17, -7, 1, -4, -9, -11, -9, -2, -7, -9, -8, -5, 2, 10, -3, -2, 4, 3, 4, 8,
    0, 8, -3, -2, -4, 0, 1, 0, -8, -5, 5, 13, 20, 12, 11, 11, 14, 13, 17, 26, 24, 28, 26, 9,
    -2, 5, 7, 11, 31, 18, 24, 18, 16, 13, 12, 9, 5, 15, 7, 13, 9, 10, 10, 9, 10, 20, 15, 11,
    10, -6, 2, 7, 10, -6, -6, -11, -19, -20, -18, -19, -19, -20, -3, 1, -2, -32, -45, -38, -21, -18, -20, -35,
    -26, -24, -24, -37, -31, -34, -37, -29, -37, -23, -13, -10, -8, -16, -25, -27, -31, -25, -24, -7, -15, -18, -19, -23,
    -22, -28, -16, -21, -24, -32, -36, -42, -38, -36, -36, -34, -32, -28, -28, -20, -15, -15, -18, -22, -28, -21, -12, -9,
    -11, -8, -20, -13, -15, -20, -8, -10, -13, -16, -13, -19, -17, -16, -20, -15, -12, -15, -2, -3, 5, 5, 0, -1,
    -8, 2, -3, -15, -26, -18, -17, -5, -9, -15, -14, -7, -6, -4, 7, 0, 3, -3, -8, -6, -1, -1, 3, 12,
    0, 16, 16, 18, 17, 7, 13, 9, 19, 8, 3, -2, 12, 12, 4, 13, 13, 22, 15, 10, 14, 25, 32, 26,
    24, 30, 39, 44, 30, 26, 39, 43, 35, 34, 29, 40, 44, 25, 25, 17, 19, 18, 15, 21, 21, 19, 7, 2,
    -2, 12, 8, 10, 20, 25, 26, 20, 7, 3, 12, 12, 16, 32, 39, 35, 15, -6, -8, 6, 6, 7, 7, 1,
    -1, -17, -4, -16, -30, -30, -47, -37, -24, -29, -25, 4, -4, -20, -31, -42, -38, -17, -22, -26, -9, -18, -6, -16,
    -22, -21, -15, -26, -32, -39, -37, -19, -13, -24, -35, -37, -37, -30, -30, -21, -14, -6, -8, 10, 17, 6, -8, -29,
    -34, -19, -6, -26, -26, -16, -24, -34, -44, -27, -14, 2, -6, -20, -20, -24, -26, -6, -1, -4, -9, -14, 3, 12,
    11, 22, 11, -2, -7, -9, -13, -6, -1, 4, 3, -7, -7, -8, 2, 4, 8, 4, 4, 6, -5, 9, 4, -5,
    2, -7, 0, 7, 2, 9, 12, 17, 6, 1, 7, 9, 19, 14, 2, 12, 12, 0, 0, 3, 7, 8, 6, -1,
    5, 11, 9, -7, -5, -2, 5, 12, -2, -8, -20, -10, -9, -17, -21, -24, -21, -18, -18, 1, -7, -9, -14, -23,
    -12, -13, -18, -28, -12, -7, -18, -21, -23, -8, 8, 3, -2, -2, -7, 0, 9, 4, -1, -24, -30, -31, -27, -28,
    -20, -32, -13, -18, -24, -22, -33, -32, -25, -27, -23, -18, -30, -19, -20, -27, -21, -29, -19, -7, -7, -16, -11, -15,
    -19, -12, -14, -11, -12, -15, -15, -22, -27, -20, -15, 0, -8, -26, -28, -19, -20, -8, -7, -4, -3, -1, -16, -4,
    2, 13, -9, -34, -30, -28, -17, -15, -17, -18, 9, 2, 2, -11, -5, 6, 22, 9, -9, -8, -6, 13, 4, 2,
    3, 8, 10, 7, 6, 18, 22, 23, 19, 17, 15, 31, 27, 20, 26, 20, 19, 26, 23, 7, 26, 18, 31, 30,
    22, 25, 42, 36, 22, 31, 30, 43, 40, 37, 40, 47, 37, 24, 26, 40, 44, 43, 35, 35, 37, 39, 40, 42,
    53, 43, 47, 44, 36, 49, 60, 46, 50, 43, 47, 56, 57, 56, 59, 72, 52, 43, 11, 18, 22, 33, 34, 30,
    28, 26, 17, 22, 22, 19, 23, 26, 22, 25, 26, 30, 40, 28, 12, 5, 19, 29, 27, 26, 24, 16, 25, 20,
    14, 16, 23, 11, 9, 9, 7, 12, 9, 3, 6, 11, 5, 12, 4, 4, 13, 18, 21, 13, 13, 2, 3, 11,
    9, 0, -12, -13, -7, -1, -10, -6, -15, -13, 0, 2, -2, -6, -8, -11, -6, -8, -11, -12, -6, -1, -2, -11,
    -1, 8, 13, 0, 1, -2, 8, 29, 32, 5, -11, -7, -3, 6, 4, 6, 3, 15, 11, 12, 18, 28, 24, 34,
    30, 30, 28, 18, 20, 16, 14, -3, -9, -7, 6, 2, -3, 10, 31, 36, 26, 21, 36, 42, 39, 13, 23, 28,
    2, -10, -13, -11, 12, 17, 12, 30, 22, 12, 6, 11, -1, 15, 17, 15, 19, 2, -5, -7, -1, 4, 18, 31,
    28, 20, 13, 10, 15, 23, 12, -1, -1, 7, 9, 9, -7, -6, -13, -13, -15, -11, -12, 0, 2, 2, 6, -9,
    -19, -13, -9, -14, -11, -3, 0, -4, -22, -18, -25, -18, -7, -5, 0, -4, -4, -11, -3, -2, -6, -11, -9, -12,
    -8, -8, -8, -13, -10, -8, 7, 10, -7, -9, -5, -2, -4, -8, -20, -9, -2, -7, -9, -21, -8, -1, -6, -8,
    -13, -9, -14, -12, -7, -7, -6, 1, 7, -8, -5, -6, -8, 4, 2, -6, 0, -9, -8, -1, 9, 6, 3, -12,
    -5, -4, -12, -2, -13, -8, -2, -13, -11, -3, -8, -8, -15, -5, -1, -5, -21, -20, -22, -10, -4, -32, -25, -16,
    1, 1, -16, -27, -34, -29, -32, -41, -35, -38, -43, -45, -48, -32, -24, -20, -33, -28, -32, -28, -27, -27, -29, -33,
    -30, -33, -30, -43, -43, -40, -35, -51, -39, -38, -34, -39, -36, -38, -48, -34, -24, -40, -47, -62, -77, -65, -52, -55,
    -51, -45, -41, -50, -45, -48, -39, -34, -26, -29, -26, -21, -6, -10, -24, -37, -44, -31, -23, -18, -20, -9, 3, 2,
    -12, -14, -18, -9, 3, -10, -6, -8, 0, 11, 5, 3, 2, -5, 16, 9, 0, 0, 4, 1, 17, 8, 8, 3,
    4, -11, -14, -7, 2, 15, 24, 9, 0, 5, 9, 23, 36, 33, 24, 17, 12, 23, 30, 26, 29, 24, 20, 26,
    25, 14, 11, 11, -5, 1, 6, 15, 0, 4, -17, 4, 22, 36, 19, 18, 15, 28, 44, 28, 30, 38, 50, 41,
    32, 41, 50, 66, 65, 55, 62, 61, 33, 26, 19, 5, 30, 21, 17, 30, 31, 12, 16, 18, 16, 34, 41, 32,
    49, 63, 65, 64, 63, 47, 49, 55, 42, 49, 62, 73, 67, 57, 42, 40, 55, 32, 39, 47, 53, 74, 74, 48,
    65, 62, 64, 57, 54, 43, 48, 53, 61, 68, 67, 64, 67, 77, 75, 64, 68, 55, 57, 70, 61, 59, 74, 58,
    55, 51, 25, 36, 32, 22, 35, 36, 34, 36, 33, 23, 34, 34, 12, 21, 22, 50, 53, 38, 7, 4, 14, 16,
    -10, -33, -34, -18, 7, -5, 8, 26, 26, 6, -8, -33, -27, -21, -33, -45, -54, -46, -59, -59, -40, -45, -24, -20,
    -18, -18, -14, -37, -53, -57, -51, -57, -68, -88, -93, -96, -85, -78, -60, -50, -64, -66, -70, -55, -50, -65, -73, -75,
    -83, -88, -102, -97, -72, -50, -62, -70, -71, -78, -61, -59, -72, -68, -63, -63, -70, -67, -66, -63, -53, -57, -71, -61,
    -51, -50, -33, -31, -40, -35, -56, -50, -35, -43, -53, -54, -32, -33, -47, -58, -56, -41, -38, -33, -42, -10, -23, -22,
    -36, -31, -29, -27, -31, -40, -20, -16, -25, -47, -65, -64, -40, -29, -31, -32, -39, -34, -39, -35, -43, -27, -7, -11,
    -16, -12, -7, 6, 6, -4, 3, 22, 8, 11, 6, -18, -6, -10, -20, -21, -12, -21, -28, -26, -35, -20, -8, -20,
    -28, -30, -31, -29, -27, -24, -8, -12, -29, -29, -19, -21, -27, -24, -11, 8, 9, -7, -19, -12, -4, -22, -29, -25,
    -23, -17, -4, -11, -6, -14, -31, -20, -6, -4, -1, 5, 10, 2, -3, 3, 10, 20, 7, 0, 3, 12, 11, 10,
    5, 7, 11, 20, 41, 43, 29, 28, 27, 18, 17, 28, 16, 30, 31, 22, 14, 22, 17, 16, 17, 14, 31, 35,
    35, 44, 33, 39, 37, 32, 27, 26, 37, 35, 37, 29, 28, 35, 34, 36, 32, 33, 41, 43, 36, 35, 37, 36,
    31, 20, 21, 20, 28, 18, 24, 38, 39, 31, 22, 14, 15, 15, 14, 8, 15, 14, 19, 14, -1, -7, -5, -4,
    2, 5, 5, 2, 6, 3, -5, 2, -4, -11, 0, -6, -11, -5, 0, 6, 6, 2, -9, 4, 2, 4, 0, -3,
    -11, -7, -10, -3, 0, -6, -3, -14, -11, -16, -5, -20, -13, -12, -10, -19, -31, -27, -24, -12, -12, -2, -14, -11,
    -25, -14, -13, 11, -4, -16, -32, -43, -22, -15, -7, -16, -8, -31, -30, -35, -38, -35, -31, -29, -44, -25, -32, -32,
    -30, -9, -27, -19, -21, -28, -15, -32, -38, -29, -15, -21, -23, -36, -19, -5, -17, -24, -27, -37, -31, -18, -25, -37,
    -48, -39, -48, -38, -45, -38, -31, -13, -32, -41, -39, -36, -10, -22, -26, -17, -16, -19, -3, -3, 6, 15, 11, -1,
    14, 0, -1, 7, 19, 19, 8, 4, 5, 7, -3, 11, 17, 17, 10, 6, 9, 10, 23, 19, 23, 46, 41, 35,
    20, -7, -15, 0, 7, 17, 38, 41, 31, 24, 11, 10, 55, 59, 58, 55, 40, 28, 31, 28, 28, 32, 26, 37,
    36, 31, 46, 64, 70, 61, 44, 37, 35, 34, 27, 21, 39, 46, 34, 34, 34, 42, 55, 43, 41, 56, 62, 56,
    58, 51, 56, 66, 37, 38, 29, 31, 29, 37, 32, 40, 52, 54, 62, 52, 43, 42, 49, 42, 45, 39, 48, 45,
    24, 14, -1, -8, 10, 18, 0, 0, -5, 11, 22, 31, 26, 42, 35, 18, 11, 4, 21, 21, 24, 2, 2, 11,
    3, -15, -27, -48, -27, 10, 0, -10, -1, 11, 30, 19, 7, -15, -10, -1, 1, -10, -20, -21, -13, 12, 27, 31,
    45, 36, 15, -1, -3, 40, 53, 31, -27, -65, -52, -19, -2, -22, -40, -51, -45, -49, -26, -8, -5, -2, -11, -13,
    -12, -3, -23, -9, 14, -2, 5, 3, -8, -4, 22, 25, 33, 17, -3, -12, 17, 38, 29, 19, 6, 12, -1, 1,
    -7, 2, 12, 3, -12, -25, 1, 10, -4, -16, -21, -38, -7, -22, -26, -7, 4, 2, 1, -7, -15, 2, -8, -1,
    -3, 0, 1, 17, 31, 37, 26, 21, 21, 19, 33, 24, 27, 26, 5, 5, -20, -31, -22, -9, -11, -5, -6, 2,
    18, 7, 13, 29, 44, 31, 40, 29, 31, 35, 21, 31, 22, 8, 27, 24, 21, 27, 28, 22, 36, 37, 28, 33,
    15, 1, -3, -3, 18, 15, 7, -1, 7, 13, 9, 11, 30, 43, 47, 37, 15, 13, 16, 12, 9, 7, 5, 0,
    1, -4, -3, 8, 3, 3, 18, 27, 34, 27, 26, 22, 13, 0, -5, -10, -19, -23, -18, -24, -26, -19, -22, -12,
    -8, -9, -10, -7, -8, -11, -12, -19, -19, -28, -35, -36, -42, -23, -4, -5, -3, 7, -2, 0, -3, -17, -15, -18,
    -14, -19, -32, -40, -50, -63, -53, -51, -47, -27, -30, -34, -33, -48, -45, -22, -16, -14, -18, -26, -20, -21, -18, -24,
    -30, -16, -13, -24, -33, -34, -30, -19, -26, -38, -33, -39, -42, -42, -39, -31, -31, -32, -28, -33, -25, -24, -27, -41,
    -42, -44, -52, -51, -43, -55, -51, -49, -47, -37, -32, -41, -31, -30, -32, -33, -44, -49, -46, -28, -18, -17, -19, -11,
    -7, 0, 1, -3, -3, -13, -13, -6, -9, -9, -11, -29, -42, -33, -39, -35, -45, -51, -44, -39, -40, -23, -5, -2,
    19, 26, 18, 18, 6, 13, 22, 37, 42, 46, 44, 31, 38, 52, 60, 54, 56, 57, 61, 75, 63, 51, 56, 51,
    38, 33, 30, 36, 48, 47, 51, 62, 82, 85, 79, 76, 79, 74, 67, 74, 76, 81, 91, 94, 87, 78, 84, 84,
    84, 80, 80, 88, 91, 83, 80, 65, 65, 75, 51, 45, 50, 48, 55, 45, 48, 54, 72, 72, 65, 72, 69, 70,
    68, 58, 29, 20, 19, 21, 28, 4, -6, 7, 9, 5, 17, 14, 21, 27, 9, 12, 15, 11, 14, 8, -1, 1,
    5, 3, -1, -2, -5, -4, -4, -7, -13, -20, -17, -16, -3, -11, -27, -46, -44, -40, -43, -35, -36, -30, -26, -23,
    -23, -24, -28, -18, -13, -13, -19, -27, -28, -27, -31, -37, -53, -56, -61, -63, -64, -62, -59, -61, -65, -70, -64, -71,
    -58, -57, -48, -43, -47, -52, -49, -42, -41, -34, -36, -30, -29, -42, -49, -50, -47, -45, -48, -63, -55, -58, -52, -54,
    -62, -76, -67, -65, -64, -69, -81, -85, -59, -65, -69, -74, -72, -72, -61, -52, -52, -41, -46, -48, -53, -64, -68, -62,
    -64, -62, -64, -61, -60, -55, -53, -59, -51, -39, -41, -39, -41, -45, -46, -37, -39, -28, -26, -32, -27, -37, -40, -31,
    -41, -44, -37, -35, -25, -22, -30, -16, -15, -24, -12, -27, -30, -33, -36, -40, -37, -45, -43, -43, -36, -30, -21, -11,
    -6, -18, -3, 11, 17, 21, 2, -1, -8, 2, 2, -3, 4, 5, 0, -5, -7, 5, 30, 30, 31, 28, 25, 19,
    14, 11, 30, 28, 37, 34, 21, 14, 23, 34, 33, 44, 50, 56, 56, 35, 20, 33, 42, 38, 34, 24, 18, 35,
    39, 35, 35, 29, 37, 47, 40, 38, 52, 50, 53, 56, 44, 40, 47, 46, 52, 51, 48, 46, 42, 17, 16, 16,
    30, 23, 26, 32, 26, 25, 23, 21, 37, 44, 36, 29, 19, 20, 23, 32, 33, 32, 29, 35, 30, 39, 41, 51,
    49, 40, 19, 31, 31, 22, 18, -2, 0, 5, 9, -1, 6, 1, 13, 17, 10, 8, 33, 47, 34, 27, 18, 14,
    10, 2, 0, 8, 13, 9, 11, 14, -4, -6, -20, -38, -39, -36, -30, -28, -32, -37, -25, -21, -12, -8, -3, 1,
    -4, 6, 4, -14, -16, -21, -20, -16, -28, -25, -9, -9, -6, -4, -10, -13, -14, -16, -9, -8, -6, 1, 0, 1,
    8, 16, 27, 23, 25, 31, 42, 45, 40, 36, 26, 18, 21, 23, 38, 51, 55, 57, 56, 53, 41, 41, 47, 42,
    41, 49, 51, 54, 44, 23, 16, 21, 34, 26, 18, 15, 21, 16, 19, 16, 4, 11, 14, 24, 24, 22, 16, 15,
    24, 23, 21, 14, 13, 11, 7, 10, 6, 8, 13, 3, 1, 0, 7, 8, 12, 10, 2, -3, -4, -6, 0, -5,
    -7, -4, -5, 2, -4, -4, -6, -1, -1, -1, -2, 6, 4, -3, -4, -18, -15, -6, -13, -11, 1, -13, -6, -7,
    -16, -12, -7, -11, -10, -10, -7, -13, -3, -3, -8, -22, -22, -20, -13, -13, -9, -10, -5, -6, -9, -14, -12, -6,
    -11, -6, -3, -10, -6, 3, -2, 1, -3, 6, 0, 0, -3, -11, -8, -10, -12, -13, -13, -14, -6, -2, -13, -12,
    -6, -9, -4, -10, -19, -24, -22, -24, -33, -29, -33, -23, -26, -24, -19, -16, -26, -19, -20, -25, -35, -37, -41, -40,
    -39, -34, -29, -36, -43, -43, -40, -49, -44, -48, -49, -45, -56, -55, -43, -51, -43, -51, -56, -59, -60, -57, -59, -56,
    -53, -56, -59, -55, -46, -45, -45, -38, -32, -34, -25, -29, -34, -38, -37, -34, -30, -23, -24, -35, -37, -36, -38, -39,
    -41, -37, -22, -17, -23, -23, -28, -33, -24, -28, -24, -14, -11, -14, -18, -15, -9, -2, -10, -10, -7, -3, -3, -5,
    -6, -1, 2, 6, 3, -5, -8, -10, -9, -11, -11, -4, 12, 17, 12, 4, 6, 11, 19, 15, 12, 13, 23, 21,
    16, 15, 19, 27, 31, 16, 7, -2, 6, 18, 18, 13, 17, 21, 26, 33, 33, 33, 37, 42, 42, 42, 40, 40,
    43, 48, 48, 47, 45, 39, 38, 38, 48, 55, 47, 38, 41, 49, 35, 27, 29, 28, 30, 28, 33, 44, 43, 41,
    38, 31, 41, 43, 37, 36, 27, 31, 24, 25, 36, 44, 38, 36, 23, 24, 40, 42, 44, 42, 32, 37, 42, 39,
    46, 43, 37, 38, 35, 15, 13, 17, 27, 24, 22, 2, 6, 6, 10, 8, 7, 10, 12, 15, 9, 13, 19, 23,
    14, 10, -10, -12, -11, -15, -24, -20, -30, -31, -25, -29, -23, -14, -20, -21, -23, -18, -8, -16, -16, -15, -19, -27,
    -34, -31, -20, -14, -5, -3, -3, -2, -1, -2, -2, -5, -11, -12, -21, -20, -26, -35, -46, -46, -48, -57, -48, -40,
    -40, -43, -46, -45, -44, -31, -26, -24, -17, -8, -12, -8, -12, -9, -4, -5, -14, -19, -15, -13, -8, -12, -10, -4,
    -15, -15, -7, 1, 10, 18, 15, 15, 11, -2, -12, -8, -1, -1, -2, 4, 8, 18, 28, 33, 22, 11, -4, 0,
    1, -3, -11, -9, -8, -9, -7, -6, 5, 5, 6, 18, 24, 29, 42, 40, 33, 28, 22, 12, 8, -7, -10, -10,
    -3, -6, -14, -10, 0, 6, 0, -1, -3, 4, 1, 3, 5, 2, 7, 5, -3, 6, 12, 8, 16, 9, -1, -5,
    -14, -20, -18, -31, -33, -22, -22, -31, -40, -40, -36, -40, -40, -46, -38, -39, -49, -42, -34, -32, -33, -35, -32, -26,
    -20, -22, -21, -8, 2, 2, -6, -16, -30, -32, -33, -31, -26, -21, -19, -15, -16, -24, -28, -31, -26, -14, -8, -9,
    -11, -2, 0, -13, -15, -7, 4, 9, 7, -2, -10, -19, -20, -14, -19, -13, -15, 3, 22, 19, 18, 19, 21, 33,
    29, 31, 40, 45, 49, 43, 39, 42, 40, 49, 54, 45, 58, 59, 57, 46, 33, 38, 46, 37, 23, 11, 10, 25,
    26, 17, 32, 42, 39, 35, 32, 25, 28, 32, 32, 32, 25, 15, 17, 15, 13, 14, 15, 27, 33, 23, 26, 29,
    30, 25, 18, 27, 31, 26, 21, 7, 5, 18, 18, 11, 0, -5, -4, 3, -2, 7, 9, 12, 13, 8, 8, 6,
    7, 4, 8, 6, 8, 3, -4, 0, -1, 1, 9, 17, 12, 19, 18, 13, 3, -1, -6, 1, 8, -1, 0, 6,
    -2, -2, -8, -3, 9, 15, 7, 11, 11, 14, 14, 9, 5, -2, -6, -8, -6, -6, -12, -12, -7, -15, -21, -16,
    -13, -10, -10, -14, -9, -12, -18, -20, -24, -18, -10, -13, -17, -25, -25, -20, -15, -13, -16, -6, -8, -17, -20, -30,
    -25, -17, -23, -29, -26, -22, -23, -30, -27, -21, -25, -31, -32, -23, -18, -18, -14, -24, -24, -17, -23, -23, -20, -24,
    -24, -29, -34, -32, -33, -34, -39, -43, -39, -39, -31, -29, -31, -31, -28, -39, -34, -34, -32, -23, -22, -24, -25, -31,
    -35, -34, -22, -20, -23, -19, -17, -28, -30, -31, -32, -30, -23, -22, -12, -19, -21, -11, -18, -16, -10, -3, 0, -10,
    -21, -28, -17, -16, -17, -17, -11, -16, -2, -4, -6, -2, -8, -6, -1, -8, -9, -2, 1, 12, 15, 13, 16, 13,
    17, 20, 17, 22, 23, 22, 20, 20, 17, 23, 13, 13, 15, 19, 24, 27, 25, 29, 24, 17, 14, 18, 13, 17,
    19, 21, 30, 34, 36, 27, 18, 26, 34, 33, 25, 25, 26, 31, 32, 22, 27, 33, 33, 35, 35, 31, 36, 40,
    33, 31, 33, 37, 35, 25, 24, 23, 38, 36, 26, 24, 27, 24, 36, 25, 24, 28, 30, 32, 31, 25, 28, 26,
    25, 21, 27, 30, 29, 21, 14, 26, 29, 17, 18, 18, 16, 18, 11, 10, 14, 21, 23, 26, 25, 16, 20, 21,
    21, 26, 22, 16, 16, 22, 15, 17, 12, 7, 20, 14, 9, 8, 3, -3, -3, 2, 8, 13, 16, 9, 9, 14,
    12, 20, 16, 6, 9, 12, 5, 7, 0, -4, 10, 3, -11, -1, 2, 1, 3, 0, -3, 1, -16, -19, -18, -6,
    -6, -14, -13, -15, -4, 3, -9, -4, -4, -11, -8, -9, -14, -8, -24, -25, -23, -25, -21, -26, -25, -19, -21, -15,
    -12, -12, -11, -19, -21, -13, -12, -14, -12, -23, -22, -25, -21, -19, -26, -25, -15, -17, -13, -16, -17, -17, -13, -16,
    -21, -16, -20, -19, -19, -24, -25, -12, -3, -8, -13, -1, 1, 3, 5, 5, 3, 4, -2, -8, -12, -10, -3, 0,
    5, 1, 5, 9, 14, 12, 18, 16, 15, 12, 10, 10, 12, 11, 10, 8, 12, 11, 7, 15, 19, 11, 20, 20,
    29, 28, 27, 24, 27, 17, 15, 21, 17, 24, 23, 16, 18, 22, 25, 21, 17, 20, 29, 29, 25, 17, 21, 18,
    10, 0, 1, -1, 12, 11, 8, 11, 7, 11, 12, 8, 4, 10, 5, 1, -4, -10, -8, -11, -15, -12, -10, 3,
    -1, -2, -4, -8, -5, -1, -11, -3, -13, -12, -23, -26, -18, -16, -21, -14, -20, -21, -12, -25, -26, -14, -10, -16,
    -24, -33, -32, -27, -38, -41, -41, -47, -45, -41, -37, -36, -33, -34, -34, -31, -42, -41, -36, -34, -32, -39, -43, -50,
    -53, -46, -37, -22, -24, -29, -28, -35, -30, -24, -27, -22, -22, -29, -30, -26, -27, -26, -24, -21, -18, -23, -20, -18,
    -16, -16, -18, -13, -23, -23, -17, -19, -19, -13, -24, -21, -15, -15, 0, -4, -9, -5, -3, 8, 11, 14, 8, 2,
    2, -4, -1, 13, 15, 15, 11, 3, -4, -4, 5, 22, 20, 18, 15, 4, 8, 7, 3, 3, 11, 13, 15, 17,
    13, 8, 6, 13, 10, 9, 20, 19, 22, 15, 5, 10, 19, 10, 8, 5, 0, 11, 13, 7, 15, 16, 22, 24,
    24, 18, 10, 7, -1, 4, 13, 12, 4, 3, -1, 1, -2, -7, -7, -1, 9, 13, 0, -6, -6, 0, 3, 3,
    1, -3, -3, 2, 0, 0, -3, -11, -13, -13, -10, -2, -1, -2, 4, 11, 7, -3, -6, -6, -8, -6, -12, -25,
    -25, -15, -9, -4, -7, -6, 3, 5, 6, 6, 2, 1, 2, 3, 3, 3, 1, 7, 11, 9, 15, 22, 21, 27,
    29, 27, 30, 31, 28, 28, 38, 32, 25, 17, 22, 21, 25, 33, 40, 50, 46, 39, 37, 35, 31, 35, 37, 43,
    44, 45, 46, 38, 32, 38, 43, 46, 51, 42, 51, 50, 37, 45, 50, 50, 59, 46, 35, 40, 38, 39, 33, 26,
    25, 29, 32, 35, 32, 34, 33, 28, 29, 28, 33, 40, 33, 31, 27, 20, 20, 17, 20, 25, 26, 27, 22, 16,
    22, 19, 14, 8, 5, 4, 4, 7, 9, 10, 4, -2, -6, -6, 0, 0, -7, -14, -10, -9, -7, -7, -13, -14,
    -12, -14, -13, -13, -10, -8, -20, -26, -35, -36, -28, -22, -30, -44, -48, -33, -32, -40, -38, -39, -39, -39, -47, -49,
    -41, -35, -40, -40, -39, -38, -39, -34, -36, -42, -41, -41, -36, -36, -40, -39, -34, -35, -35, -45, -45, -38, -35, -38,
    -46, -44, -38, -39, -40, -41, -38, -29, -36, -36, -38, -34, -33, -32, -37, -26, -26, -33, -38, -38, -30, -29, -24, -29,
    -27, -27, -24, -27, -33, -27, -26, -20, -18, -20, -18, -24, -23, -24, -21, -28, -19, -13, -13, -11, -13, -9, -12, -10,
    -5, -5, -3, -1, 2, 0, 3, -5, -12, -14, -5, 2, 0, 2, 2, -1, 5, 3, -1, 1, 5, 6, 5, 7,
    6, 15, 10, 4, 8, 12, 15, 11, 5, 6, 18, 18, 11, 16, 9, 10, 15, 10, 12, 16, 19, 17, 13, 4,
    -1, 1, 6, 9, 12, 8, -3, 6, 6, 6, 8, 11, 14, 17, 15, 16, 11, 6, 8, 14, 10, 3, 2, 1,
    7, 10, 1, 3, 9, 14, 17, 9, 1, 5, 9, 9, 2, 0, 7, 2, -5, -9, -1, 10, 9, 3, 1, 2,
    8, 3, -2, -2, 0, 2, -6, -10, -14, -18, -19, -19, -21, -11, -6, -7, -1, -5, -6, -2, -2, -10, -9, -13,
    -17, -23, -20, -10, -8, -10, -8, -5, -10, -2, -2, 5, 12, 16, 8, 1, -6, -6, -5, -2, -4, -3, 0, -6,
    -1, 7, 6, 3, 3, -2, 1, -4, -5, 2, 1, -7, -9, -11, -9, -3, 1, -5, 1, -2, -2, 1, -1, 0,
    -1, -5, -11, -4, -4, -2, -9, -9, -15, -12, -8, -12, -4, 2, 0, -1, -8, -8, -11, -2, -8, -10, -12, -6,
    -5, -14, -8, -15, -8, -7, -1, -3, 0, 7, 5, -1, -9, -4, -2, 7, -3, -7, -6, -9, 1, 3, 6, 4,
    5, -3, -5, 3, 3, 0, 6, 9, 16, 12, 2, 5, 6, 5, 6, 13, 16, 16, 20, 20, 17, 15, 17, 15,
    20, 18, 23, 20, 17, 21, 16, 14, 19, 17, 16, 17, 21, 18, 26, 29, 28, 21, 25, 31, 27, 22, 19, 27,
    30, 31, 22, 14, 13, 19, 29, 24, 25, 20, 20, 14, 17, 14, 19, 21, 20, 23, 18, 16, 11, 10, 17, 22,
    25, 19, 12, 10, 13, 18, 16, 12, 6, 16, 11, 8, 3, -7, -7, 9, 14, 7, 6, 6, -4, 2, 4, 6,
    8, 10, 4, 2, -1, 4, 5, 4, 3, 7, 7, 10, 3, -6, -2, 4, 7, -3, -13, -17, -15, -6, -9, -13,
    -15, -15, -13, -9, -18, -11, -14, -16, -1, -2, -16, -25, -19, -16, -13, -19, -25, -22, -20, -25, -35, -32, -34, -22,
    -21, -23, -22, -18, -13, -14, -23, -15, -11, -8, -6, -12, -16, -15, -16, -16, -19, -17, -8, -10, -11, -11, -14, -11,
    -16, -17, -17, -19, -12, -14, -20, -24, -23, -23, -14, -10, -2, -1, 0, -5, -10, -7, -1, 1, -8, -15, -22, -19,
    -16, -16, -11, -8, -7, -4, -12, -8, 0, 5, -1, -4, -7, -16, -12, -3, -3, -3, -7, -5, -4, 0, 1, 5,
    9, 8, 9, 1, 1, 3, 2, -2, -12, -10, 1, 1, 5, 10, 16, 11, 1, -2, 1, 5, 15, 18, 6, -2,
    -8, -6, -3, -1, 1, 6, 6, 3, 6, 7, 12, 9, 6, 11, 13, 12, 5, 6, 11, 14, 16, 8, 13, 4,
    15, 17, 15, 15, 6, 3, 6, 12, 6, 8, 7, 10, 7, 1, 3, -3, 5, 6, 8, 1, 2, 8, 6, 5,
    9, 10, 8, 14, 12, 18, 18, 9, 10, 9, 4, 10, 9, 4, 4, 4, 9, 14, 13, 7, 9, 5, 10, 12,
    6, 4, 15, 10, 8, 8, 10, 15, 13, 7, 4, 3, -4, -6, -4, -4, 3, 0, -9, -11, -5, -3, -4, -4,
    -7, -2, 1, -4, -5, -8, -14, -16, -15, -15, -23, -22, -21, -13, -15, -25, -16, -14, -13, -25, -18, -20, -18, -20,
    -21, -14, -21, -16, -22, -23, -22, -22, -24, -22, -22, -21, -18, -20, -22, -22, -25, -24, -24, -22, -27, -24, -21, -22,
    -26, -35, -24, -26, -19, -14, -10, -13, -16, -24, -21, -14, -13, -13, -16, -24, -30, -26, -26, -30, -23, -13, -16, -20,
    -23, -26, -18, -13, -3, -5, -7, -9, -7, -10, -12, -5, -7, -1, -11, -16, -12, -10, -14, -6, -4, -4, 4, 2,
    2, 2, 5, 2, 2, -8, 2, 6, 2, -7, -8, -9, -5, 0, 7, 6, 6, 7, 8, 6, 10, 12, 9, 2,
    1, -3, -5, 4, 7, 2, 0, 10, 3, 3, 11, 13, 16, 19, 15, 9, 7, 4, 9, 10, 6, 10, 6, 0,
    6, 7, 7, 11, 13, 17, 23, 17, 18, 15, 22, 24, 22, 17, 15, 23, 26, 24, 17, 13, 21, 28, 27, 23,
    16, 17, 13, 16, 19, 17, 20, 25, 15, 11, 12, 19, 24, 24, 22, 14, 9, 11, 3, 0, 7, 10, 10, 6,
    9, 8, 8, 9, 7, -2, 0, 7, 8, 6, 5, 0, 1, 0, 2, 4, 9, 5, 3, -1, -1, -6, -15, -14,
    -16, -19, -18, -22, -23, -14, -20, -17, -15, -19, -17, -24, -22, -20, -20, -20, -23, -24, -27, -28, -30, -27, -21, -21,
    -22, -21, -22, -23, -25, -25, -21, -31, -33, -32, -28, -25, -31, -31, -27, -25, -22, -26, -20, -19, -15, -17, -17, -7,
    -16, -11, -8, -12, -7, -2, -6, -4, -7, -11, -6, -9, -1, 4, 11, 16, 15, 14, 11, 18, 24, 20, 30, 26,
    22, 30, 23, 25, 24, 33, 32, 32, 35, 34, 36, 33, 37, 37, 42, 35, 32, 28, 30, 31, 31, 39, 35, 38,
    37, 40, 36, 34, 31, 33, 40, 38, 33, 34, 43, 38, 45, 45, 52, 49, 50, 37, 38, 30, 24, 20, 30, 30,
    27, 27, 28, 25, 26, 31, 36, 33, 38, 35, 24, 34, 29, 23, 18, 12, 10, 4, 12, 20, 11, 14, 16, 20,
    26, 25, 27, 29, 25, 19, 11, 4, 9, 7, 6, -2, -7, -2, -3, 8, 6, 7, 3, -4, -5, -9, 1, 2,
    -4, -8, -17, -23, -14, -8, -4, -2, -5, -16, -13, -7, -6, -11, -18, -28, -36, -32, -37, -34, -29, -24, -25, -26,
    -21, -13, -12, -11, -12, -15, -22, -23, -20, -13, -9, -10, -14, -16, -17, -20, -15, -7, -9, -12, -12, -12, -15, -11,
    -17, -22, -20, -17, -14, -23, -26, -25, -15, -3, -12, -12, -15, -12, -3, -8, -10, -5, -2, 1, -2, -4, -3, -4,
    -2, -2, -3, -8, -15, -11, -4, -3, 6, 15, 15, 12, 6, 10, 16, 21, 20, 14, 11, 14, 14, 9, 13, 21,
    20, 22, 18, 18, 16, 25, 29, 27, 30, 29, 29, 31, 33, 29, 32, 30, 24, 27, 34, 34, 45, 45, 46, 40,
    48, 51, 48, 43, 39, 46, 38, 36, 32, 29, 29, 26, 18, 21, 19, 23, 27, 33, 30, 33, 25, 24, 28, 28,
    23, 12, 15, 17, 17, 19, 17, 20, 25, 35, 44, 40, 37, 29, 22, 18, 13, 9, 4, 0, -7, -11, -20, -22,
    -34, -30, -20, -17, -14, -11, -11, -18, -20, -27, -22, -19, -21, -27, -31, -41, -38, -38, -38, -39, -38, -29, -37, -40,
    -45, -39, -40, -40, -43, -49, -48, -54, -56, -51, -51, -52, -44, -46, -49, -63, -54, -49, -41, -34, -34, -37, -39, -44,
    -44, -42, -46, -44, -37, -36, -40, -43, -42, -45, -37, -32, -29, -25, -21, -15, -15, -22, -24, -22, -16, -15, -11, -4,
    -15, -18, -35, -26, -25, -16, -14, -24, -16, -8, -10, -18, -21, -22, -12, -4, -8, -7, -8, -11, -11, -5, 2, 1,
    1, -3, -4, -7, 0, 8, 6, 4, -4, -5, -7, -13, -10, -5, -2, -6, -7, -12, -10, -3, 1, -1, 4, 3,
    -5, 1, 7, 13, 21, 21, 9, 11, 11, 15, 6, -2, -2, -3, 5, 1, 0, 6, 8, 4, 9, 6, 20, 18,
    17, 22, 18, 19, 17, 18, 9, 19, 20, 17, 5, 6, 4, 15, 13, 9, 10, 7, 13, 10, 7, 10, 17, 13,
    12, 4, 6, 7, 16, 8, 18, 16, 16, 13, 18, 21, 27, 31, 20, 19, 12, 15, 10, 9, 9, 14, 10, 13,
    7, 14, 9, 11, 18, 22, 23, 22, 19, 8, 9, 1, 3, -2, 1, -1, 1, 8, 2, -1, 7, 10, 10, 10,
    10, 4, 1, 5, 9, 8, -1, 0, -1, -3, -12, -16, -8, -4, -3, -3, -15, -18, -15, -14, -3, -4, -7, -3,
    -2, -12, -17, -18, -13, -9, -5, -4, -14, -13, -15, -6, -5, 7, 6, 5, 0, -14, -28, -26, -25, -23, -22, -22,
    -19, -24, -23, -19, -10, -3, 2, 5, 1, -2, -4, -6, -5, -14, -13, -19, -6, -8, -10, -13, -7, 0, 5, 8,
    6, 4, -1, -7, -8, -10, -7, -5, -11, -4, 7, -6, -15, -9, -9, -5, -6, -8, -9, -10, -10, -12, -13, -14,
    -9, -6, -9, -9, -6, -12, -5, -4, -13, -12, -19, -10, -8, -2, -10, -6, -4, -8, -11, -4, 5, 8, 11, 2,
    4, 0, 7, -4, -1, -4, -7, -8, -12, -12, -12, -13, -10, -4, -9, -6, -7, -2, 2, 3, 3, -3, -6, 0,
    7, 3, 0, -5, -1, -4, -5, -14, -13, -11, -7, -2, -2, -2, -1, 7, 2, 5, 8, 11, 10, 7, 11, 9,
    7, 10, 7, 8, 12, 15, 11, 8, 3, 9, 10, 9, 4, -4, 5, 6, 4, -6, -7, -2, 6, 3, 1, 7,
    10, 14, 5, 9, 9, 7, 7, 7, 8, 10, 4, -3, -3, -3, 6, 5, 5, 6, 8, 12, 9, 10, 7, 7,
    11, 4, 2, 7, 5, 3, -2, 1, 11, 11, 11, 5, 2, 1, -2, 4, -5, -2, 3, -6, -13, -18, -13, -19,
    -16, -21, -15, -15, -14, -19, -14, -12, -14, -11, -14, -4, -3, 3, -3, 5, 0, -2, -5, -7, -6, -1, -8, -11,
    -12, -10, -8, -8, -11, -15, -17, -22, -27, -32, -29, -22, -27, -27, -26, -19, -17, -12, -8, -8, 2, -3, 0, -5,
    -1, 2, 0, -8, -13, -8, 4, 10, 11, 19, 20, 14, -3, -12, -4, -1, -16, -18, -24, -11, -11, -14, -8, -5,
    17, 19, 12, 11, 28, 32, 31, 15, 9, 6, 17, 21, 13, 18, 12, 4, 8, 15, 18, 11, 12, 7, 5, 2,
    -18, -13, -10, 3, 3, -10, -11, -11, -5, -5, -4, -7, 4, 14, 17, 12, 17, 23, 28, 31, 16, 10, 12, 23,
    18, 17, 10, 13, 12, 10, 12, 10, 19, 21, 22, 18, 19, 12, 25, 28, 23, 25, 27, 29, 24, 26, 27, 32,
    29, 26, 25, 33, 31, 30, 33, 27, 31, 32, 32, 32, 40, 40, 33, 24, 26, 32, 35, 35, 30, 36, 37, 44,
    44, 43, 40, 41, 35, 36, 30, 29, 31, 31, 34, 26, 26, 24, 34, 34, 38, 30, 30, 34, 27, 32, 26, 30,
    30, 26, 27, 26, 29, 31, 30, 30, 27, 27, 26, 33, 33, 19, 2, 6, 6, 0, 3, -3, 8, 12, 13, 3,
    3, 2, 2, 11, 14, 13, 10, 5, 3, -2, -7, -5, -12, -12, -8, -13, -13, -16, -22, -23, -22, -17, -18, -23,
    -26, -28, -29, -37, -35, -36, -28, -30, -33, -33, -40, -34, -33, -46, -39, -35, -37, -34, -44, -42, -51, -49, -49, -53,
    -48, -45, -45, -41, -46, -49, -45, -46, -35, -43, -39, -39, -35, -37, -47, -40, -38, -43, -46, -54, -46, -43, -57, -54,
    -54, -56, -55, -61, -57, -47, -53, -55, -59, -61, -54, -57, -53, -61, -50, -51, -56, -61, -56, -54, -53, -52, -54, -50,
    -44, -47, -64, -62, -62, -51, -52, -46, -42, -42, -38, -38, -41, -36, -27, -32, -34, -36, -26, -28, -27, -31, -27, -17,
    -13, -22, -21, -11, -2, -14, -17, -7, -6, -6, -11, -8, -6, -3, -2, 0, 2, 6, 11, 5, 18, 18, 18, 8,
    10, 11, 10, 21, 21, 23, 18, 21, 21, 27, 23, 29, 30, 28, 24, 11, 19, 21, 33, 28, 21, 16, 17, 17,
    21, 22, 27, 29, 22, 21, 21, 25, 23, 20, 18, 22, 20, 14, 13, 10, 19, 23, 23, 21, 15, 19, 16, 19,
    15, 3, -2, 4, 4, 13, 13, 12, 15, 19, 15, 18, 22, 27, 27, 24, 15, 22, 19, 22, 27, 25, 39, 24,
    29, 20, 14, 20, 16, 15, 21, 13, 12, 13, 7, 7, 3, 12, 9, 6, 20, 20, 22, 13, 13, 8, 3, 13,
    2, 5, 8, -3, 5, -3, 11, 10, 7, 19, 15, 21, 17, 13, 13, 20, 8, 6, 1, -1, 6, 6, 6, 3,
    11, 20, 20, 18, 18, 17, 15, 13, 6, 10, 17, 14, 16, 10, 7, 2, 9, 9, 22, 15, 17, 6, 4, 6,
    3, 8, 10, 7, 3, 4, -3, -6, -5, 4, -8, -6, -8, 0, 0, -9, -3, -5, 1, -1, -4, -4, 8, 9,
    7, -4, 1, -4, -2, 6, 5, 15, 8, 1, -2, -4, 2, 3, 12, 7, 12, 11, 7, 9, 7, 11, 14, 13,
    17, 11, 15, 10, 13, 26, 24, 32, 35, 31, 29, 26, 21, 27, 31, 31, 29, 32, 30, 29, 26, 26, 32, 30,
    24, 37, 31, 23, 24, 17, 25, 29, 22, 26, 29, 29, 30, 23, 25, 30, 30, 25, 14, 15, 16, 26, 20, 19,
    17, 29, 25, 14, 19, 20, 25, 23, 13, 8, 13, 13, 11, 8, 13, 17, 22, 13, 7, 3, 2, 0, 4, -4,
    -6, -7, -5, 2, 7, 14, 13, 13, 15, 10, 17, 12, 16, 13, 11, 10, 1, 2, 10, 8, 0, -1, 2, 8,
    4, 2, -9, 3, -3, 5, -1, 1, 15, 15, 18, 3, -3, 3, 6, 5, -3, -8, -4, -3, -12, -8, -19, -5,
    -5, -1, -5, -4, -5, -1, -7, -3, 3, -3, 2, -14, -6, -9, -5, -12, -13, -11, -5, -14, -13, -14, -18, -18,
    -27, -28, -32, -25, -23, -24, -20, -18, -37, -26, -41, -29, -24, -33, -30, -43, -45, -37, -33, -17, -25, -29, -23, -30,
    -23, -27, -35, -30, -32, -34, -38, -37, -35, -48, -38, -40, -41, -34, -35, -35, -31, -37, -41, -40, -41, -30, -20, -23,
    -19, -25, -31, -28, -32, -23, -14, -18, -15, -18, -15, -19, -16, -14, -8, -5
};
/* @@SOUND_DATA_END@@ */
