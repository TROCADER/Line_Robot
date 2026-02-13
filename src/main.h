#include <avr/io.h>

#pragma once
void init_pins();

void init_rtc();

void init_adc();
void init_tca();

void init_pidA();

void read_sensor(uint8_t sensor, uint16_t *sensor_data);