#pragma once

#include <stdint.h>

void echo_init(void);
uint16_t echo_read(void);
uint16_t echo_to_cm(uint16_t pulse_us);