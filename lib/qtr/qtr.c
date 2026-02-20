#include "qtr.h"
#include "../../src/pins.h"
#include <avr/io.h>
#include <util/delay.h>

#define ON true
#define OFF false

static const uint8_t qtr_masks[QTR_SENSOR_COUNT] = {
    PIN0_bm, PIN1_bm, PIN2_bm, PIN3_bm, PIN4_bm,
};

void qtr_init(void)
{
    PORTD.DIRCLR = QTR_SENSOR_MASK;
    PORTD.OUTCLR = QTR_SENSOR_MASK;
    PORTD.DIRSET = QTR_EMITTER_MASK;
    PORTD.OUTCLR = QTR_EMITTER_MASK;
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

    uint16_t time = 0;
    uint8_t remaining = QTR_SENSOR_MASK;

    while (remaining != 0 && time < QTR_MAX_TIME)
    {
        uint8_t pins = PORTD.IN & remaining;
        uint8_t went_low = (uint8_t)(~pins) & remaining;

        if (went_low)
        {
            for (uint8_t i = 0; i < QTR_SENSOR_COUNT; i++)
            {
                uint8_t mask = qtr_masks[i];
                if (went_low & mask)
                {
                    values[i] = time;
                    remaining &= (uint8_t)~mask;
                }
            }
        }

        time++;
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
