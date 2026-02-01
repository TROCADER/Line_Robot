#include "main.h"
#include "../lib/pid/pid.h"
#include "pins.h"
#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdlib.h>

ISR(RTC_PIT_vect)
{
    // TODO: READ SENSORS AND APPLY PID CONTROL
    RTC.PITINTFLAGS = RTC_PI_bm; // Clear interrupt flag
}

int main(void)
{
    init_pins();

    init_rtc();
    init_adc();

    return 0;
}

void init_adc()
{
    PORTD.PIN0CTRL = PORT_ISC_INPUT_DISABLE_gc;
    PORTD.PIN1CTRL = PORT_ISC_INPUT_DISABLE_gc;

    ADC0.CTRLA = 0 << ADC_CONVMODE_bp | ADC_RESSEL_12BIT_gc | ADC_ENABLE_bm;
    ADC0.CTRLB = ADC_SAMPNUM_NONE_gc;
    ADC0.CTRLC = ADC_PRESC_DIV20_gc; // 4MHz/20 = 200KHz TODO: Probably needs tuning
    ADC0.CTRLD = ADC_INITDLY_DLY0_gc | ADC_SAMPDLY_DLY0_gc;
    ADC0.CTRLE = ADC_WINCM0_bp;
    ADC0.SAMPCTRL = ADC_SAMPLEN0_bp;
    ADC0.MUXPOS = ADC_MUXPOS_AIN0_gc;
    ADC0.MUXNEG = ADC_MUXNEG_GND_gc;
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
    // PORTD.DIR = 0 | PIN_LEFT_IR_ANALOG | PIN_RIGHT_IR_ANALOG;
    // PORTD.IN = 0 | PIN_LEFT_IR_ANALOG | PIN_RIGHT_IR_ANALOG;
}