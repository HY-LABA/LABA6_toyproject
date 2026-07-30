#ifndef ENCODER_PIO_H
#define ENCODER_PIO_H

#include <stdint.h>

void encoder_init_all(void);
int32_t encoder_get_count(int motor_index);

#endif  // ENCODER_PIO_H
