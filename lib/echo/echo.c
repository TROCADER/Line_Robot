#include "echo.h"
#include "../../src/pins.h"
#include <avr/io.h>
#include <util/delay.h>

#define ECHO_TIMEOUT_US 30000U
#define ECHO_TICKS_PER_US 2 // 4MHz / ((prescaler) 2 * 10^6) = 2 
#define ECHO_TIMEOUT_TICKS (ECHO_TIMEOUT_US * ECHO_TICKS_PER_US)

void echo_init()
{
    PORTC.DIRSET = ECHO_TRIG_MASK;
    PORTC.OUTCLR = ECHO_TRIG_MASK;
    PORTC.DIRCLR = ECHO_ECHO_MASK;
    PORTC.OUTCLR = ECHO_ECHO_MASK;

    TCB1.CTRLA = 0;
    TCB1.CTRLB = TCB_CNTMODE_INT_gc;
    TCB1.CCMP = 0xFFFF;
    TCB1.CNT = 0;
    TCB1.INTCTRL = 0;
    TCB1.INTFLAGS = TCB_CAPT_bm;
    TCB1.CTRLA = TCB_CLKSEL_DIV2_gc | TCB_ENABLE_bm;
}

uint16_t echo_read()
{
    PORTC.OUTCLR = ECHO_TRIG_MASK;
    _delay_us(2);
    PORTC.OUTSET = ECHO_TRIG_MASK;
    _delay_us(10);
    PORTC.OUTCLR = ECHO_TRIG_MASK;

    TCB1.CNT = 0;
    while (((PORTC.IN & ECHO_ECHO_MASK) == 0U) && (TCB1.CNT < ECHO_TIMEOUT_TICKS))
    {
    }

    if ((PORTC.IN & ECHO_ECHO_MASK) == 0U)
    {
        return 0U;
    }

    TCB1.CNT = 0;
    while (((PORTC.IN & ECHO_ECHO_MASK) != 0U) && (TCB1.CNT < ECHO_TIMEOUT_TICKS))
    {
    }

    uint16_t pulse_ticks = TCB1.CNT;
    uint16_t pulse_us = (pulse_ticks / ECHO_TICKS_PER_US);

    return pulse_us;
}

uint16_t echo_to_cm(uint16_t pulse_us)
{
    if (pulse_us == 0U)
    {
        return 0U;
    }

    return (pulse_us / 58U);
}

float echo_power_scale(uint16_t pulse_us)
{
    if (pulse_us < 1000)
    {
        float scale = (float)pulse_us / 1000.0f;
        
        if (scale > 0.25)
        {
            return scale;
        }
        return 0.0;
    }
    
    return 1.0;
}