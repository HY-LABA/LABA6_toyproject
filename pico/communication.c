// [START 0xAA][LEN][PAYLOAD][CHECKSUM], CHECKSUM=sum(PAYLOAD)%256. pi5/communication.py와 일치해야 함.
#include "communication.h"

#include <string.h>

#include "pico/stdlib.h"

#define FRAME_START 0xAA
#define SEND_PAYLOAD_LEN 12
#define RECV_PAYLOAD_LEN 24

typedef enum {
    RX_WAIT_START,
    RX_WAIT_LEN,
    RX_WAIT_PAYLOAD,
    RX_WAIT_CHECKSUM,
} RxState;

static RxState rx_state = RX_WAIT_START;
static uint8_t rx_len = 0;
static uint8_t rx_index = 0;
static uint8_t rx_payload[SEND_PAYLOAD_LEN];

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

DriveCommand communication_try_receive(void) {
    DriveCommand result = {0};

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
                rx_state = (rx_len == SEND_PAYLOAD_LEN) ? RX_WAIT_PAYLOAD : RX_WAIT_START;
                break;

            case RX_WAIT_PAYLOAD:
                rx_payload[rx_index++] = byte;
                if (rx_index >= rx_len) {
                    rx_state = RX_WAIT_CHECKSUM;
                }
                break;

            case RX_WAIT_CHECKSUM:
                if (checksum(rx_payload, rx_len) == byte) {
                    // PAYLOAD = <fff> target_vx, target_vy, timeout_s.
                    // 바이트 수는 예전(좌표+구동시간)과 같지만 의미가 다르다 —
                    // pi5/communication.py와 반드시 같이 봐야 한다.
                    memcpy(&result.target_vx, &rx_payload[0], 4);
                    memcpy(&result.target_vy, &rx_payload[4], 4);
                    memcpy(&result.timeout_s, &rx_payload[8], 4);
                    result.valid = true;
                }
                rx_state = RX_WAIT_START;
                return result;
        }
    }

    return result;
}

void communication_send_odometry(RobotPose pose, RobotVelocity velocity) {
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
