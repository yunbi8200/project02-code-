/* touch_algo.c - Smart Ink STM32 #1 좌표 계산
 * 순서: 트림평균 -> 보정(0~4095) -> SETTLE/TAIL -> 적응형 EMA -> 속도
 * 모든 계산은 정수 (touch_sim.py와 같은 결과가 나오는지 PC에서 검증함)
 */
#include "touch_algo.h"

static ta_cfg_t cfg;

static int32_t since;                                   /* 연속으로 눌린 횟수 */
static int32_t dl_x[TA_TAIL + 1], dl_y[TA_TAIL + 1], dl_i;   /* TAIL 지연 버퍼 */
static int32_t fvalid, sx, sy, spd;                     /* 필터 상태 (x256 고정소수점) */
static int32_t hx[TA_SPEED_WIN + 1], hy[TA_SPEED_WIN + 1], h_i, h_cnt;  /* 속도용 기록 */
static uint16_t last_x, last_y;

void ta_init(const ta_cfg_t *c)
{
    cfg = *c;
    since = 0; dl_i = 0; fvalid = 0; h_i = 0; h_cnt = 0;
    last_x = 0; last_y = 0;
}

/* 16개 정렬 -> 가운데 8개 평균 (튀는 값 제거) */
uint16_t ta_trim_mean16(const uint16_t *b)
{
    uint16_t s[16];
    int i, j;
    for (i = 0; i < 16; i++) s[i] = b[i];
    for (i = 1; i < 16; i++) {                 /* 삽입 정렬 */
        uint16_t v = s[i];
        for (j = i - 1; j >= 0 && s[j] > v; j--) s[j + 1] = s[j];
        s[j + 1] = v;
    }
    uint32_t sum = 0;
    for (i = 4; i < 12; i++) sum += s[i];
    return (uint16_t)(sum >> 3);
}

static int32_t calib(int32_t raw, int32_t lo, int32_t hi)
{
    int32_t c = (raw - lo) * TA_COORD_MAX / (hi - lo);
    if (c < 0) c = 0;
    if (c > TA_COORD_MAX) c = TA_COORD_MAX;
    return c;
}

static uint32_t isqrt32(uint32_t n)
{
    uint32_t r = 0, bit = 1UL << 30;
    while (bit > n) bit >>= 2;
    while (bit) {
        if (n >= r + bit) { n -= r + bit; r = (r >> 1) + bit; }
        else r >>= 1;
        bit >>= 2;
    }
    return r;
}

static void no_output(ta_out_t *o)
{
    fvalid = 0; h_cnt = 0;
    o->touch = 0; o->speed = 0;
    o->x = last_x; o->y = last_y;
}

void ta_step(uint8_t touched, const uint16_t *bx, const uint16_t *by, ta_out_t *o)
{
    if (!touched) { since = 0; no_output(o); return; }
    since++;

    /* 1. 대표값 + 보정 */
    o->raw_x = ta_trim_mean16(bx);
    o->raw_y = ta_trim_mean16(by);
    dl_x[dl_i] = calib(o->raw_x, cfg.cal_xmin, cfg.cal_xmax);
    dl_y[dl_i] = calib(o->raw_y, cfg.cal_ymin, cfg.cal_ymax);

    /* 2. TAIL만큼 지난 값을 꺼냄 (그 사이 펜이 떨어지면 since가 0이 되어 버려짐) */
    int32_t r = (dl_i + 1) % (TA_TAIL + 1);
    int32_t xi = dl_x[r], yi = dl_y[r];
    dl_i = (dl_i + 1) % (TA_TAIL + 1);
    if (since <= TA_SETTLE + TA_TAIL) { no_output(o); return; }

    /* 3. 적응형 EMA: 느리면 강하게, 빠르면 약하게 */
    xi <<= 8; yi <<= 8;
    if (!fvalid) {
        sx = xi; sy = yi; spd = 0; fvalid = 1;      /* 펜 닿을 때마다 리셋 */
    } else {
        int32_t v  = spd < cfg.v_ref ? spd : cfg.v_ref;
        int32_t sh = cfg.sh_slow - (cfg.sh_slow - cfg.sh_fast) * v / cfg.v_ref;
        int32_t nx = sx + ((xi - sx) >> sh);
        int32_t ny = sy + ((yi - sy) >> sh);
        int32_t dx = nx - sx, dy = ny - sy;
        int32_t inst = ((dx < 0 ? -dx : dx) + (dy < 0 ? -dy : dy)) * TA_FS >> 8;
        spd += (inst - spd) >> 3;
        sx = nx; sy = ny;
    }
    int32_t fx = (sx + 128) >> 8, fy = (sy + 128) >> 8;

    /* 4. 속도: 10ms 전 위치와의 거리 (mm/s -> 0~255) */
    hx[h_i] = fx; hy[h_i] = fy;
    int32_t old = (h_i + 1) % (TA_SPEED_WIN + 1);
    h_i = (h_i + 1) % (TA_SPEED_WIN + 1);
    if (h_cnt < TA_SPEED_WIN + 1) h_cnt++;
    uint8_t sp = 0;
    if (h_cnt == TA_SPEED_WIN + 1) {
        int32_t ddx = fx - hx[old], ddy = fy - hy[old];
        uint32_t dx_um = (uint32_t)(ddx < 0 ? -ddx : ddx) * 155080u / TA_COORD_MAX;  /* 패널 폭 155.08mm */
        uint32_t dy_um = (uint32_t)(ddy < 0 ? -ddy : ddy) * 86920u / TA_COORD_MAX;   /* 패널 높이 86.92mm */
        if (dx_um > 46000u) dx_um = 46000u;                                         /* 제곱 합 32비트 안에 */
        if (dy_um > 46000u) dy_um = 46000u;
        uint32_t mm_s  = isqrt32(dx_um * dx_um + dy_um * dy_um) / 10u;       /* 10ms 동안 um -> mm/s */
        uint32_t s = mm_s * 255u / 400u;
        sp = (uint8_t)(s > 255u ? 255u : s);
    }

    o->x = last_x = (uint16_t)fx;
    o->y = last_y = (uint16_t)fy;
    o->touch = 1;
    o->speed = sp;
}

/* [AA][xL][xH][yL][yH][touch][speed][checksum] */
void ta_make_packet(const ta_out_t *o, uint8_t p[8])
{
    p[0] = 0xAA;
    p[1] = (uint8_t)(o->x & 0xFF); p[2] = (uint8_t)(o->x >> 8);
    p[3] = (uint8_t)(o->y & 0xFF); p[4] = (uint8_t)(o->y >> 8);
    p[5] = o->touch;
    p[6] = o->speed;
    p[7] = (uint8_t)(p[0] + p[1] + p[2] + p[3] + p[4] + p[5] + p[6]);
}
