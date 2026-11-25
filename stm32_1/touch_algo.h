/* touch_algo.h - Smart Ink STM32 #1 좌표 계산 (하드웨어 무관, PC에서도 컴파일됨) */
#ifndef TOUCH_ALGO_H
#define TOUCH_ALGO_H
#include <stdint.h>

#define TA_BURST      16    /* 축당 사용 샘플 수 */
#define TA_SETTLE     5     /* 펜 닿은 직후 버릴 측정 횟수 */
#define TA_TAIL       3     /* 펜 뗄 때 꼬리 제거용 지연 (측정 횟수) */
#define TA_FS         1000  /* 측정 주기 Hz */
#define TA_SPEED_WIN  10    /* 속도 계산 간격 (측정 횟수) */
#define TA_COORD_MAX  4095  /* 출력 좌표 범위 0~4095 */

typedef struct {
    int32_t cal_xmin, cal_xmax;   /* 보정: 왼쪽/오른쪽 끝 원시값 (바꿔 넣으면 방향 반전) */
    int32_t cal_ymin, cal_ymax;   /* 보정: 위/아래 끝 원시값 */
    int32_t sh_slow, sh_fast;     /* 적응형 EMA: 느릴 때 1/2^6, 빠를 때 1/2^1 */
    int32_t v_ref;                /* 이 속도(좌표단위/초) 이상이면 가장 약하게 */
} ta_cfg_t;

typedef struct {
    uint16_t x, y;        /* 0~4095 (필터 적용) */
    uint8_t  touch;       /* 1: 유효한 좌표 */
    uint8_t  speed;       /* 0~255 (0~400mm/s) */
    uint16_t raw_x, raw_y;/* 트림평균 원시값 (보정값 측정용) */
} ta_out_t;

void     ta_init(const ta_cfg_t *cfg);
uint16_t ta_trim_mean16(const uint16_t *buf);
void     ta_step(uint8_t touched, const uint16_t *bx, const uint16_t *by, ta_out_t *out);
void     ta_make_packet(const ta_out_t *o, uint8_t pkt[8]);

#endif
