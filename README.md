# SMART INK

임베디드시스템설계 Project 2 (4조) 소스 코드. STM32F103RB (Nucleo-F103RB), 레지스터 직접 제어.

| 폴더 | 보드 | 역할 |
|---|---|---|
| `stm32_1/` | STM32 #1 | 4선식 터치패널 좌표 측정 (ADC+DMA) → SPI2 마스터로 #3에 송신 |
| `stm32_2/` | STM32 #2 | FSR 필압 측정 (TIM3→ADC1→DMA) → USART3로 #3에 송신 |
| `stm32_3/` | STM32 #3 | 좌표·필압·모드 수신 및 통합 → USART2로 PC 송신, 필압 연동 필기음 (PWM+DMA) |
| `arduino/` | Arduino Uno | 버튼으로 PEN/ERASER 전환, RGB LED 표시 → SoftwareSerial(D3, 19200)로 #3에 모드 송신 |
| `common/` | 공통 | 보드 간 패킷 규격 `smartink_protocol.h` |

각 보드 코드는 STM32CubeIDE 프로젝트의 `Core/Src`, `Core/Inc`에 넣어 빌드한다.
Arduino 스케치는 `common/smartink_protocol.h`를 스케치 폴더에 복사한 뒤 빌드한다.
