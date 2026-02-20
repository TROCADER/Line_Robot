#include "qtr.h"
#include "../../src/pins.h"
#include <avr/io.h>
#include <util/delay.h>

#define ON true
#define OFF false

#define QTR_TICKS_PER_US 2 // 4MHz / ((prescaler) 2 * 10^6) = 2 
#define QTR_TIMEOUT_TICKS (QTR_MAX_TIME * QTR_TICKS_PER_US)

static const uint8_t qtr_masks[QTR_SENSOR_COUNT] = {
    PIN0_bm, PIN1_bm, PIN2_bm, PIN3_bm, PIN4_bm,
};

void qtr_init()
{
    PORTD.DIRCLR = QTR_SENSOR_MASK;
    PORTD.OUTCLR = QTR_SENSOR_MASK;
    PORTD.DIRSET = QTR_EMITTER_MASK;
    PORTD.OUTCLR = QTR_EMITTER_MASK;

    TCB0.CTRLA = 0;
    TCB0.CTRLB = TCB_CNTMODE_INT_gc;
    TCB0.CCMP = 0xFFFF;
    TCB0.CNT = 0;
    TCB0.INTCTRL = 0;
    TCB0.INTFLAGS = TCB_CAPT_bm;
    TCB0.CTRLA = TCB_CLKSEL_DIV2_gc | TCB_ENABLE_bm;
}

void qtr_set_emitters(bool on)
{
    if (on)
    {
        PORTD.OUTSET = QTR_EMITTER_MASK;
    }
    else
    {
        PORTD.OUTCLR = QTR_EMITTER_MASK;
    }
}

void qtr_read(uint16_t values[], bool emitters_on)
{
    if (emitters_on)
    {
        qtr_set_emitters(ON);
    }

    PORTD.DIRSET = QTR_SENSOR_MASK;
    PORTD.OUTSET = QTR_SENSOR_MASK;
    _delay_us(20); // < 10 us

    PORTD.DIRCLR = QTR_SENSOR_MASK;

    uint8_t remaining = QTR_SENSOR_MASK;
    uint16_t time_us = 0;

    TCB0.CNT = 0;

    while (remaining != 0 && TCB0.CNT < QTR_TIMEOUT_TICKS)
    {
        time_us = (TCB0.CNT / QTR_TICKS_PER_US);
        uint8_t pins = PORTD.IN & remaining;
        uint8_t went_low = (~pins) & remaining;

        if (went_low)
        {
            for (uint8_t i = 0; i < QTR_SENSOR_COUNT; i++)
            {
                uint8_t mask = qtr_masks[i];
                if (went_low & mask)
                {
                    values[i] = time_us;
                    remaining &= ~mask;
                }
            }
        }
    }

    if (remaining)
    {
        for (uint8_t i = 0; i < QTR_SENSOR_COUNT; i++)
        {
            uint8_t mask = qtr_masks[i];
            if (remaining & mask)
            {
                values[i] = QTR_MAX_TIME;
            }
        }
    }

    if (emitters_on)
    {
        qtr_set_emitters(OFF);
    }
}
