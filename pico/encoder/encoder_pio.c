#include "encoder_pio.h"

#include "hardware/gpio.h"
#include "hardware/pio.h"

#include "encoder.pio.h"  // CMake(pico_generate_pio_header)가 encoder.pio에서 자동 생성
#include "../config.h"

static PIO pio_inst[NUM_MOTORS];
static uint sm_inst[NUM_MOTORS];
static int32_t last_count[NUM_MOTORS];

void encoder_init_all(void) {
    uint offset = pio_add_program(pio0, &quadrature_1x_program);

    for (int i = 0; i < NUM_MOTORS; i++) {
        pio_inst[i] = pio0;
        sm_inst[i] = i;
        last_count[i] = 0;

        int pin_a = MOTOR_PINS[i].enc_a;
        int pin_b = MOTOR_PINS[i].enc_b;

        pio_gpio_init(pio0, pin_a);
        pio_gpio_init(pio0, pin_b);
        // 풀업 유지: 오픈드레인 엔코더면 필수이고, 푸시풀이면 내부 ~50k 풀업은
        // 출력 드라이브에 비해 무시할 수준이라 해가 없다. 양쪽 다 안전한 기본값.
        gpio_pull_up(pin_a);
        gpio_pull_up(pin_b);

        pio_sm_config c = quadrature_1x_program_get_default_config(offset);
        sm_config_set_in_pins(&c, pin_a);
        sm_config_set_jmp_pin(&c, pin_b);

        pio_sm_init(pio0, i, offset, &c);
        pio_sm_set_enabled(pio0, i, true);
    }
}

int32_t encoder_get_count(int motor_index) {
    PIO pio = pio_inst[motor_index];
    uint sm = sm_inst[motor_index];

    while (!pio_sm_is_rx_fifo_empty(pio, sm)) {
        last_count[motor_index] = (int32_t)pio_sm_get(pio, sm);
    }
    return last_count[motor_index];
}
