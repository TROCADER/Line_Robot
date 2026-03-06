#pragma once

#include <stdint.h>

void echo_init();
uint16_t echo_read();
uint16_t echo_to_cm(uint16_t pulse_us);
float echo_power_scale(uint16_t pulse_us);