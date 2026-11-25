/*
 * smartink_protocol.h
 *
 *  Created on: 2026. 11. 19.
 *      Author: se601
 */
#ifndef SRC_SMARTINK_PROTOCOL_H_
#define SRC_SMARTINK_PROTOCOL_H_
/*
 * smartink_protocol.h
 * SMART INK 보드 간 공통 패킷 규격 (STM32 #1, #2, #3 공용)
 * 이 파일 하나만 #include 하면 된다 (.c 파일 필요 없음).
 *
 *   +------+------+------+-----+--------------+------+
 *   | 0xAA | 0x55 | TYPE | LEN | PAYLOAD[LEN] | CRC8 |
 *   +------+------+------+-----+--------------+------+
 *
 *   - 2바이트 값은 little-endian
 *   - CRC8: 다항식 0x07, 초기값 0x00, TYPE ~ PAYLOAD 끝까지
 */
#ifndef SMARTINK_PROTOCOL_H
#define SMARTINK_PROTOCOL_H
#include <stdint.h>
#define PKT_SOF1            0xAAu
#define PKT_SOF2            0x55u
#define PKT_MAX_PAYLOAD     32u
#define PKT_MAX_FRAME       (PKT_MAX_PAYLOAD + 5u)  /* SOF1 SOF2 TYPE LEN ... CRC */
#define PKT_TYPE_TOUCH      0x01u   /* STM32 #1 -> #3 (SPI) */
#define PKT_TYPE_PRESSURE   0x02u   /* STM32 #2 -> #3 */
#define PKT_TYPE_MODE       0x03u   /* Arduino -> #3 (UART 19200) */
#define PKT_TYPE_SENSOR     0x10u   /* STM32 #3 -> PC/웹 */
#define PKT_LEN_TOUCH       6u
#define PKT_LEN_PRESSURE    4u
#define PKT_LEN_MODE        3u
#define PKT_LEN_SENSOR      10u
#define PKT_FRAME_MODE      (PKT_LEN_MODE + 5u)     /* 모드 패킷 8바이트 */
#define MODE_PEN            0u
#define MODE_ERASER         1u
#define MODE_OPT_MUTE       0x01u   /* OPTIONS 비트 0, 나머지 비트는 0 */
/* SENSOR flags */
#define SENSOR_FLAG_DRAW         0x01u  /* touch && press */
#define SENSOR_FLAG_TOUCH_STALE  0x02u  /* #1 데이터 끊김 */
#define SENSOR_FLAG_PRESS_STALE  0x04u  /* #2 데이터 끊김 */
#define SENSOR_FLAG_RX_ERROR     0x08u  /* 직전 패킷 이후 수신 오류 */
#define SENSOR_FLAG_ERASER       0x10u  /* Arduino 모드: 1 = 지우개, 0 = 펜 */
#define SENSOR_FLAG_MODE_STALE   0x20u  /* Arduino 200 ms 동안 정상 수신 없음 (DRAW 도 꺼짐) */
#define SENSOR_FLAG_MUTE         0x40u  /* Arduino 음소거 */
typedef struct {
    uint8_t  seq;
    uint16_t x;
    uint16_t y;
    uint8_t  touch;
} touch_data_t;
typedef struct {
    uint8_t  seq;
    uint16_t pressure;
    uint8_t  press;
} pressure_data_t;
typedef struct {
    uint8_t  seq;
    uint8_t  mode;       /* MODE_PEN / MODE_ERASER */
    uint8_t  options;    /* MODE_OPT_MUTE */
} mode_data_t;
typedef struct {
    uint8_t  seq;
    uint16_t x;
    uint16_t y;
    uint16_t pressure;
    uint8_t  press;
    uint8_t  touch;
    uint8_t  flags;
} sensor_data_t;
/* 바이트 스트림 파서. 0으로 초기화된 상태로 그대로 쓰면 된다 */
typedef struct {
    uint8_t  state;
    uint8_t  type;
    uint8_t  len;
    uint8_t  idx;
    uint8_t  crc;
    uint8_t  payload[PKT_MAX_PAYLOAD];
    uint32_t ok_count;
    uint32_t errors;     /* CRC 불일치 + 비정상 LEN */
} pkt_parser_t;
/* ------------------------------------------------------------------ */
/* 내부 헬퍼                                                           */
/* ------------------------------------------------------------------ */
static inline void pkt_put_u16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)(v >> 8);
}
static inline uint16_t pkt_get_u16(const uint8_t *p)
{
    return (uint16_t)(p[0] | ((uint16_t)p[1] << 8));
}
static inline uint8_t crc8_update(uint8_t crc, uint8_t data)
{
    crc ^= data;
    for (uint8_t i = 0; i < 8; i++) {
        crc = (crc & 0x80u) ? (uint8_t)((crc << 1) ^ 0x07u) : (uint8_t)(crc << 1);
    }
    return crc;
}
/* ------------------------------------------------------------------ */
/* 송신: out 에 프레임을 쓰고 길이를 반환                               */
/* ------------------------------------------------------------------ */
static inline uint16_t pkt_build(uint8_t type, const uint8_t *payload, uint8_t len, uint8_t *out)
{
    if (len > PKT_MAX_PAYLOAD) {
        return 0;
    }
    uint16_t n = 0;
    out[n++] = PKT_SOF1;
    out[n++] = PKT_SOF2;
    out[n++] = type;
    out[n++] = len;
    uint8_t crc = crc8_update(crc8_update(0, type), len);
    for (uint8_t i = 0; i < len; i++) {
        out[n++] = payload[i];
        crc = crc8_update(crc, payload[i]);
    }
    out[n++] = crc;
    return n;
}
static inline uint16_t pkt_build_touch(
    const touch_data_t *d, uint8_t *out)
{
    uint8_t p[PKT_LEN_TOUCH];
    p[0] = d->seq;
    pkt_put_u16(&p[1], d->x);
    pkt_put_u16(&p[3], d->y);
    p[5] = d->touch ? 1u : 0u;
    return pkt_build(PKT_TYPE_TOUCH, p, PKT_LEN_TOUCH, out);
}
static inline uint16_t pkt_build_pressure(
    const pressure_data_t *d, uint8_t *out)
{
    uint8_t p[PKT_LEN_PRESSURE];
    p[0] = d->seq;
    pkt_put_u16(&p[1], d->pressure);
    p[3] = d->press ? 1u : 0u;
    return pkt_build(PKT_TYPE_PRESSURE, p, PKT_LEN_PRESSURE, out);
}
static inline uint16_t pkt_build_mode(const mode_data_t *d, uint8_t *out)
{
    uint8_t p[PKT_LEN_MODE] = { d->seq, d->mode, d->options };
    return pkt_build(PKT_TYPE_MODE, p, PKT_LEN_MODE, out);
}
static inline uint16_t pkt_build_sensor(const sensor_data_t *d, uint8_t *out)
{
    uint8_t p[PKT_LEN_SENSOR];
    p[0] = d->seq;
    pkt_put_u16(&p[1], d->x);
    pkt_put_u16(&p[3], d->y);
    pkt_put_u16(&p[5], d->pressure);
    p[7] = d->press;
    p[8] = d->touch;
    p[9] = d->flags;
    return pkt_build(PKT_TYPE_SENSOR, p, PKT_LEN_SENSOR, out);
}
/* ------------------------------------------------------------------ */
/* 수신: 성공 0, 길이 불일치 -1                                        */
/* ------------------------------------------------------------------ */
static inline int pkt_decode_touch(
    const uint8_t *payload, uint8_t len, touch_data_t *out)
{
    if (len != PKT_LEN_TOUCH) {
        return -1;
    }
    out->seq   = payload[0];
    out->x     = pkt_get_u16(&payload[1]);
    out->y     = pkt_get_u16(&payload[3]);
    out->touch = payload[5] ? 1u : 0u;
    return 0;
}
static inline int pkt_decode_pressure(
    const uint8_t *payload, uint8_t len, pressure_data_t *out)
{
    if (len != PKT_LEN_PRESSURE) {
        return -1;
    }
    out->seq      = payload[0];
    out->pressure = pkt_get_u16(&payload[1]);
    out->press    = payload[3] ? 1u : 0u;
    return 0;
}
/* MODE 가 0/1 이 아니면 -1 (out 은 그대로) */
static inline int pkt_decode_mode(const uint8_t *payload, uint8_t len, mode_data_t *out)
{
    if (len != PKT_LEN_MODE || payload[1] > MODE_ERASER) {
        return -1;
    }
    out->seq     = payload[0];
    out->mode    = payload[1];
    out->options = payload[2];
    return 0;
}
/* 1바이트씩 넣는다. 정상 프레임 완성 시 1 (p->type/len/payload 유효), 아니면 0 */
static inline int pkt_parser_feed(pkt_parser_t *p, uint8_t b)
{
    enum { PS_SOF1 = 0, PS_SOF2, PS_TYPE, PS_LEN, PS_PAYLOAD, PS_CRC };
    switch (p->state) {
    case PS_SOF1:
        if (b == PKT_SOF1) {
            p->state = PS_SOF2;
        }
        break;
    case PS_SOF2:
        if (b == PKT_SOF2) {
            p->state = PS_TYPE;
        } else if (b != PKT_SOF1) {       /* 0xAA 0xAA 0x55 는 계속 대기 */
            p->state = PS_SOF1;
        }
        break;
    case PS_TYPE:
        p->type  = b;
        p->crc   = crc8_update(0, b);
        p->state = PS_LEN;
        break;
    case PS_LEN:
        if (b > PKT_MAX_PAYLOAD) {        /* 비정상 길이 -> 버리고 재동기화 */
            p->errors++;
            p->state = PS_SOF1;
            break;
        }
        p->len   = b;
        p->idx   = 0;
        p->crc   = crc8_update(p->crc, b);
        p->state = (b == 0) ? PS_CRC : PS_PAYLOAD;
        break;
    case PS_PAYLOAD:
        p->payload[p->idx++] = b;
        p->crc = crc8_update(p->crc, b);
        if (p->idx >= p->len) {
            p->state = PS_CRC;
        }
        break;
    default: /* PS_CRC */
        p->state = PS_SOF1;
        if (b == p->crc) {
            p->ok_count++;
            return 1;
        }
        p->errors++;
        break;
    }
    return 0;
}
#endif /* SMARTINK_PROTOCOL_H */
#endif /* SRC_SMARTINK_PROTOCOL_H_ */
