/*
 * SMART INK · Arduino Uno (ATmega328P)
 *
 * 기능:
 * 1. 버튼으로 PEN / ERASER 모드 전환
 * 2. RGB LED로 현재 모드 표시
 * 3. UART로 STM32 #3에 MODE 패킷 전송
 * 4. USB Serial Monitor로 디버깅
 *
 * UART 연결:
 * Arduino D3 (TX) -> STM32 USART1 RX (PA10)
 * Arduino GND     -> STM32 GND
 *
 * 주의: Uno TX는 5V이므로 3.3V 레벨 변환 필요
 */

#include <Arduino.h>
#include <string.h>
#include <SoftwareSerial.h>
#include "smartink_protocol.h"

// ========== 핀 설정 ==========
const uint8_t BUTTON_PIN = 2;

const uint8_t LED_R_PIN = 9;
const uint8_t LED_G_PIN = 10;
const uint8_t LED_B_PIN = 11;

// STM32 UART 연결
const uint8_t STM_RX_PIN = 4;  // 사용하지 않아도 됨
const uint8_t STM_TX_PIN = 3;  // STM32로 MODE 전송

SoftwareSerial stmUART(STM_RX_PIN, STM_TX_PIN);

// ========== 통신 설정 ==========
const unsigned long SERIAL_BAUD = 115200UL;
const unsigned long STM_UART_BAUD = 19200UL;

const unsigned long DEBOUNCE_MS = 30UL;
const unsigned long MODE_REPEAT_MS = 2000UL;

// STM32 MODE_STALE(200ms)보다 짧게 설정
const unsigned long MODE_UART_PERIOD_MS = 50UL;

const bool COMMON_ANODE = false;
const bool MUTE_ENABLED = false;

// ========== RGB 색상 ==========
const uint8_t PEN_RGB[3] = {0, 180, 0};
const uint8_t ERASER_RGB[3] = {255, 70, 0};

// ========== 모드 상태 ==========
bool eraserMode = false;

bool modePending = true;
bool uartPending = true;
bool uartSentOnce = false;
bool ledDirty = true;

uint8_t modeSeq = 0;

unsigned long lastModeSentAt = 0;
unsigned long lastUARTSentAt = 0;

unsigned long uartPacketCount = 0;

// ========== 버튼 상태 ==========
bool rawButton = HIGH;
bool stableButton = HIGH;
bool buttonArmed = true;

unsigned long rawChangedAt = 0;

// ========== Serial 명령 ==========
char commandBuffer[32];
uint8_t commandLength = 0;
bool commandOverflow = false;

// ========== 함수 선언 ==========
void updateButton();
void updateLED();
void processSerialCommand();
void sendMode();
void sendModeUART();

// ===================================
// SETUP
// ===================================

void setup() {

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  pinMode(LED_R_PIN, OUTPUT);
  pinMode(LED_G_PIN, OUTPUT);
  pinMode(LED_B_PIN, OUTPUT);

  rawButton = stableButton = digitalRead(BUTTON_PIN);
  buttonArmed = (stableButton == HIGH);

  rawChangedAt = millis();

  // USB 시리얼 모니터
  Serial.begin(SERIAL_BAUD);

  // STM32 UART
  stmUART.begin(STM_UART_BAUD);

  updateLED();

  // 초기 모드 전송
  sendMode();
  sendModeUART();

  Serial.println(F("SMART INK UART READY"));
}

// ===================================
// LOOP
// ===================================

void loop() {

  updateButton();

  updateLED();

  processSerialCommand();

  // STM32로 UART 전송
  sendModeUART();

  // USB 디버그 모드 반복 출력
  if ((unsigned long)(millis() - lastModeSentAt)
      >= MODE_REPEAT_MS) {

    modePending = true;
  }

  sendMode();

  // 1초마다 UART 전송 상태 확인
  static unsigned long lastDebugAt = 0;

  if ((unsigned long)(millis() - lastDebugAt)
      >= 1000UL) {

    lastDebugAt = millis();

    if (Serial.availableForWrite() >= 40) {

      Serial.print(F("UART_TX_COUNT="));
      Serial.println(uartPacketCount);

      Serial.print(F("CURRENT_MODE="));

      if (eraserMode) {
        Serial.println(F("ERASER"));
      } else {
        Serial.println(F("PEN"));
      }
    }
  }
}

// ===================================
// 버튼 처리
// ===================================

void updateButton() {

  const unsigned long now = millis();

  const bool reading = digitalRead(BUTTON_PIN);

  if (reading != rawButton) {

    rawButton = reading;
    rawChangedAt = now;
  }

  if (reading == stableButton ||
      (unsigned long)(now - rawChangedAt) < DEBOUNCE_MS) {
    return;
  }

  stableButton = reading;

  if (stableButton == HIGH) {

    buttonArmed = true;

  } else if (buttonArmed) {

    buttonArmed = false;

    // PEN <-> ERASER
    eraserMode = !eraserMode;

    ledDirty = true;

    modePending = true;

    // UART 즉시 전송 요청
    uartPending = true;
  }
}

// ===================================
// RGB LED
// ===================================

void updateLED() {

  if (!ledDirty) return;

  const uint8_t *rgb =
      eraserMode ? ERASER_RGB : PEN_RGB;

  analogWrite(
    LED_R_PIN,
    COMMON_ANODE ? 255 - rgb[0] : rgb[0]
  );

  analogWrite(
    LED_G_PIN,
    COMMON_ANODE ? 255 - rgb[1] : rgb[1]
  );

  analogWrite(
    LED_B_PIN,
    COMMON_ANODE ? 255 - rgb[2] : rgb[2]
  );

  ledDirty = false;
}

// ===================================
// USB 시리얼 명령 처리
// ===================================

void processSerialCommand() {

  for (uint8_t budget = 0;
       budget < 32 && Serial.available() > 0;
       ++budget) {

    const char ch = (char)Serial.read();

    if (ch == '\n') {

      if (!commandOverflow) {

        commandBuffer[commandLength] = '\0';

        if (strcmp(commandBuffer, "GET_MODE") == 0) {

          modePending = true;
        }
      }

      commandLength = 0;
      commandOverflow = false;

    } else if (ch == '\r') {

      // 무시

    } else if (!commandOverflow) {

      if (ch < 32 ||
          ch > 126 ||
          commandLength >= sizeof(commandBuffer) - 1) {

        commandOverflow = true;

      } else {

        commandBuffer[commandLength++] = ch;
      }
    }
  }
}

// ===================================
// USB 시리얼 모니터 출력
// ===================================

void sendMode() {

  if (!modePending) return;

  const char *message =
      eraserMode ? "MODE=ERASER\n" : "MODE=PEN\n";

  const size_t count = strlen(message);

  if (Serial.availableForWrite() < (int)count) {
    return;
  }

  Serial.write((const uint8_t *)message, count);

  lastModeSentAt = millis();

  modePending = false;
}

// ===================================
// STM32 UART MODE 전송
// ===================================

void sendModeUART() {

  const unsigned long now = millis();

  // 모드 변경이 없다면 50ms 주기로 재전송
  if (!uartPending && uartSentOnce &&
      (unsigned long)(now - lastUARTSentAt)
          < MODE_UART_PERIOD_MS) {

    return;
  }

  // MODE 데이터 생성
  mode_data_t data = {

    modeSeq,

    (uint8_t)(
      eraserMode ? MODE_ERASER : MODE_PEN
    ),

    (uint8_t)(
      MUTE_ENABLED ? MODE_OPT_MUTE : 0u
    )
  };

  // 기존 프로토콜 패킷 생성
  uint8_t frame[PKT_FRAME_MODE];

  const uint16_t size =
      pkt_build_mode(&data, frame);

  if (size == 0 || size > sizeof(frame)) {
    return;
  }

  // UART 전송
  const size_t written = stmUART.write(frame, size);

  if (written == size) {

    modeSeq++;

    uartPacketCount++;

    uartSentOnce = true;

    lastUARTSentAt = millis();

    uartPending = false;

  } else {

    uartPending = true;
  }
}
