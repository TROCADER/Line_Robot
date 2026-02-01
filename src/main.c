#include "main.h"
#include "../lib/pid/pid.h"
#include "pins.h"
#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdlib.h>

ISR(RTC_PIT_vect)
{
    // READ SENSORS AND APPLY PID CONTROL
    RTC.PITINTFLAGS = RTC_PI_bm; // Clear interrupt flag
}

int main(void)
{
    init_pins();

    init_rtc();

    return 0;
}

void init_rtc()
{
    RTC.CTRLA = RTC_PRESCALER_DIV1_gc;
    RTC.CLKSEL = CLKSEL_OSC32K_gc;
    RTC.PITCTRLA = RTC_PITEN_bm;
    RTC.PITINTCTRL = RTC_PI_bm;
}

void init_pins()
{
    // Initialize all pins
    // PIN A - Motors
    PORTA.DIR = 0 | PIN_LEFT_MOTOR_A | PIN_LEFT_MOTOR_B | PIN_RIGHT_MOTOR_A | PIN_RIGHT_MOTOR_B;
    PORTA.OUT = 0 | PIN_LEFT_MOTOR_A | PIN_LEFT_MOTOR_B | PIN_RIGHT_MOTOR_A | PIN_RIGHT_MOTOR_B;

    // PIN D - Sensors
    PORTD.DIR = 0 | PIN_LEFT_IR_ANALOG | PIN_RIGHT_IR_ANALOG;
    PORTD.IN = 0 | PIN_LEFT_IR_ANALOG | PIN_RIGHT_IR_ANALOG;
}