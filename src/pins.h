#pragma once

#define QTR_SENSOR_COUNT 5
#define QTR_MAX_TIME 3000

// QTR sensors on PORTD pins 0-4, emitter control on PORTD pin 5
#define QTR_SENSOR_MASK (PIN0_bm | PIN1_bm | PIN2_bm | PIN3_bm | PIN4_bm)
#define QTR_EMITTER_MASK PIN5_bm

#define ECHO_TRIG_MASK PIN0_bm
#define ECHO_ECHO_MASK PIN1_bm