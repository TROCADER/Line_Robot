#include "echo.h"
#include "../../src/pins.h"
#include <avr/io.h>
#include <util/delay.h>

#define ECHO_TIMEOUT_US 30000U
#define ECHO_TICKS_PER_US 2U
#define ECHO_TIMEOUT_TICKS (ECHO_TIMEOUT_US * ECHO_TICKS_PER_US)

void echo_init(void)
{
    PORTC.DIRSET = ECHO_TRIG_MASK;
    PORTC.OUTCLR = ECHO_TRIG_MASK;
    PORTC.DIRCLR = ECHO_ECHO_MASK;
    PORTC.OUTCLR = ECHO_ECHO_MASK;

    TCB0.CTRLA = 0;
    TCB0.CTRLB = TCB_CNTMODE_INT_gc;
    TCB0.CCMP = 0xFFFF;
    TCB0.CNT = 0;
    TCB0.INTCTRL = 0;
    TCB0.INTFLAGS = TCB_CAPT_bm;
    TCB0.CTRLA = TCB_CLKSEL_DIV2_gc | TCB_ENABLE_bm;
}

uint16_t echo_read(void)
{
    PORTC.OUTCLR = ECHO_TRIG_MASK;
    _delay_us(2);
    PORTC.OUTSET = ECHO_TRIG_MASK;
    _delay_us(10);
    PORTC.OUTCLR = ECHO_TRIG_MASK;

    TCB0.CNT = 0;
    while (((PORTC.IN & ECHO_ECHO_MASK) == 0U) && (TCB0.CNT < ECHO_TIMEOUT_TICKS))
    {
    }

    if ((PORTC.IN & ECHO_ECHO_MASK) == 0U)
    {
        return 0U;
    }

    TCB0.CNT = 0;
    while (((PORTC.IN & ECHO_ECHO_MASK) != 0U) && (TCB0.CNT < ECHO_TIMEOUT_TICKS))
    {
    }

    uint16_t pulse_ticks = TCB0.CNT;
    uint16_t pulse_us = (uint16_t)(pulse_ticks / ECHO_TICKS_PER_US);

    return pulse_us;
}

uint16_t echo_to_cm(uint16_t pulse_us)
{
    if (pulse_us == 0U)
    {
        return 0U;
    }

    return (uint16_t)(pulse_us / 58U);
}