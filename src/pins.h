#pragma once

// WE ONLY MOVE FORWARDS
#define PIN_LEFT_MOTOR_A 1  // PA0
#define PIN_RIGHT_MOTOR_A 2 // PA1

#define QTR_SENSOR_COUNT 5
#define QTR_MAX_TIME 3000

// QTR sensors on PORTD pins 0-4, emitter control on PORTD pin 5
#define QTR_SENSOR_MASK (PIN0_bm | PIN1_bm | PIN2_bm | PIN3_bm | PIN4_bm)
#define QTR_EMITTER_MASK PIN5_bm