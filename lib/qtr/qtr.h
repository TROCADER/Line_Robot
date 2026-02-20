#pragma once

#include <stdbool.h>
#include <stdint.h>

void qtr_init(void);
void qtr_set_emitters(bool on);
void qtr_read(uint16_t values[], bool emitters_on);