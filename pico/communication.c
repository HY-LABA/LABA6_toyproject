// [START 0xAA][LEN][PAYLOAD][CHECKSUM], CHECKSUM=sum(PAYLOAD)%256.
// 페이로드 두 종류, LEN으로 구분한다 (docs/protocol.md 2장, pi5/communication.py와 일치해야 함):
//   16B <ffff> target_x, target_y, time_remaining_s, timeout_s — 평상시 구동(목표점)
//   12B <fff>  target_vx, target_vy, timeout_s                — 텔레옵·정지(속도)
#include "communication.h"

#include <string.h>

#include "pico/stdlib.h"
#include "tusb.h"   // tud_cdc_* — 송신 여유 확인용(논블로킹)

#define FRAME_START 0xAA
#define TARGET_PAYLOAD_LEN 16
#define VELOCITY_PAYLOAD_LEN 12
#define RECV_PAYLOAD_LEN 24
#define MAX_RX_PAYLOAD_LEN TARGET_PAYLOAD_LEN

typedef enum {
    RX_WAIT_START,
    RX_WAIT_LEN,
    RX_WAIT_PAYLOAD,
    RX_WAIT_CHECKSUM,
} RxState;

static RxState rx_state = RX_WAIT_START;
static uint8_t rx_len = 0;
static uint8_t rx_index = 0;
static uint8_t rx_payload[MAX_RX_PAYLOAD_LEN];

static uint8_t checksum(const uint8_t *data, uint8_t len) {
    uint32_t sum = 0;
    for (uint8_t i = 0; i < len; i++) {
        sum += data[i];
    }
    return (uint8_t)(sum % 256);
}

void communication_init(void) {
    rx_state = RX_WAIT_START;
    rx_index = 0;
}

RxCommand communication_try_receive(void) {
    RxCommand result = {0};
    result.kind = RX_CMD_NONE;

    int c;
    while ((c = getchar_timeout_us(0)) != PICO_ERROR_TIMEOUT) {
        uint8_t byte = (uint8_t)c;

        switch (rx_state) {
            case RX_WAIT_START:
                if (byte == FRAME_START) {
                    rx_state = RX_WAIT_LEN;
                }
                break;

            case RX_WAIT_LEN:
                rx_len = byte;
                rx_index = 0;
                // 12든 16이든 받는다 — 그 외 길이는 예전처럼 조용히 버린다
                // (옛 펌웨어에 새 프레임을 보내도 안전하게 무시되던 성질 유지).
                if (rx_len == TARGET_PAYLOAD_LEN || rx_len == VELOCITY_PAYLOAD_LEN) {
                    rx_state = RX_WAIT_PAYLOAD;
                } else {
                    rx_state = RX_WAIT_START;
                }
                break;

            case RX_WAIT_PAYLOAD:
                rx_payload[rx_index++] = byte;
                if (rx_index >= rx_len) {
                    rx_state = RX_WAIT_CHECKSUM;
                }
                break;

            case RX_WAIT_CHECKSUM:
                if (checksum(rx_payload, rx_len) == byte) {
                    if (rx_len == TARGET_PAYLOAD_LEN) {
                        memcpy(&result.target_x, &rx_payload[0], 4);
                        memcpy(&result.target_y, &rx_payload[4], 4);
                        memcpy(&result.time_remaining_s, &rx_payload[8], 4);
                        memcpy(&result.timeout_s, &rx_payload[12], 4);
                        result.kind = RX_CMD_TARGET;
                    } else {
                        memcpy(&result.target_vx, &rx_payload[0], 4);
                        memcpy(&result.target_vy, &rx_payload[4], 4);
                        memcpy(&result.timeout_s, &rx_payload[8], 4);
                        result.kind = RX_CMD_VELOCITY;
                    }
                }
                // ★ 여기서 return 하지 않는다 (2026-09-04 수정).
                //   예전에는 프레임 하나를 완성하면 즉시 돌아갔다. 그러면 이 함수는
                //   **제어 틱당 최대 1프레임**만 소비한다 — 제어 주기가 20ms(50Hz)인데
                //   파이는 카메라 프레임마다(30~60Hz) 보내므로, 파이가 50Hz를 넘는
                //   순간 USB CDC 버퍼에 밀리기 시작하고 **지연이 무한히 누적된다.**
                //   낡은 목표점으로 달리는 건 오도메트리가 낡은 것과 똑같이 위험하다.
                //
                //   그래서 버퍼를 끝까지 비우고 **마지막 완전 프레임만** 채택한다.
                //   중간 프레임들은 어차피 낡은 정보다. 파이 쪽
                //   `try_receive_odometry()` 가 반대 방향에서 쓰는 방침과 같다
                //   (docs/protocol.md 4장).
                rx_state = RX_WAIT_START;
                break;
        }
    }

    return result;
}

void communication_send_odometry(RobotPose pose, RobotVelocity velocity) {
    // ★ 논블로킹 (2026-09-04 추가). `putchar_raw` 는 호스트가 안 읽으면 **블로킹한다**
    //   — 제어 루프 안에서 걸리면 모터가 마지막 듀티로 계속 돈다. 오도메트리는 한
    //   프레임 빠져도 되지만 제어 루프는 멈추면 안 된다 (docs/protocol.md 4장,
    //   pico/TODO.md 3장). 보낼 자리가 없으면 이번 주기는 그냥 건너뛴다.
    if (!tud_cdc_connected() ||
        tud_cdc_write_available() < (uint32_t)(RECV_PAYLOAD_LEN + 3)) {
        return;
    }

    uint8_t payload[RECV_PAYLOAD_LEN];
    memcpy(&payload[0], &pose.x, 4);
    memcpy(&payload[4], &pose.y, 4);
    memcpy(&payload[8], &pose.theta, 4);
    memcpy(&payload[12], &velocity.vx, 4);
    memcpy(&payload[16], &velocity.vy, 4);
    memcpy(&payload[20], &velocity.omega, 4);

    putchar_raw(FRAME_START);
    putchar_raw(RECV_PAYLOAD_LEN);
    for (int i = 0; i < RECV_PAYLOAD_LEN; i++) {
        putchar_raw(payload[i]);
    }
    putchar_raw(checksum(payload, RECV_PAYLOAD_LEN));
}
