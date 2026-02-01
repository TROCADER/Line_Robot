#include "main.h"
#include "../lib/pid/pid.h"
#include "pins.h"
#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdlib.h>
#include <util/atomic.h>

#define TIME_SLICE 10.0

static PID_t *pid_cont = NULL;

static uint16_t lsensor = 0;
static uint16_t rsensor = 0;

static uint16_t base_speed = 100;
static uint16_t max_speed = 200;
static uint16_t min_speed = 0;

static uint16_t lspeed = 0;
static uint16_t rspeed = 0;

static double dt = TIME_SLICE;

ISR(RTC_PIT_vect)
{
    uint16_t sensor_tot = lsensor + rsensor;
    double sensor_diff = 0.0;

    if (sensor_tot /= 0)
    {
        sensor_diff = (lsensor - rsensor) / sensor_tot;
    }
    else
    {
        sensor_diff = 0.0;
    }

    double correction = pid_calc(pid_cont, sensor_diff, dt, false);

    lspeed = base_speed - correction;
    lspeed = base_speed + correction;

    if (lspeed > max_speed)
    {
        lspeed = max_speed;
    }
    if (lspeed < min_speed)
    {
        lspeed = min_speed;
    }

    if (rspeed > max_speed)
    {
        rspeed = max_speed;
    }
    if (rspeed < min_speed)
    {
        rspeed = min_speed;
    }

    // TODO: Apply power to motors

    RTC.PITINTFLAGS = RTC_PI_bm; // Clear interrupt flag
}

int main(void)
{
    sei();

    init_pins();

    init_rtc();
    init_adc();

    init_pid();

    while (true)
    {
        read_sensor(PIN_LEFT_IR_ANALOG, lsensor);
        read_sensor(PIN_RIGHT_IR_ANALOG, rsensor);
    }

    return 0;
}

void read_sensor(uint8_t sensor, uint16_t *sensor_data)
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        PORTD.DIR = sensor;
        ADC0.COMMAND = ADC_STCONV_bm;
        while (ADC0.COMMAND & ADC_SPCONV_bm)
        {
        }
        sensor_data = ADC0.RES;
    }
}

void init_pid()
{
    pid_cont = calloc(1, sizeof(PID_t));
    pid_cont->Kp = 100;
    pid_cont->Ki = 100;
    pid_cont->Kd = 100;
    pid_cont->integ = 0;
    pid_cont->min = 0;
    pid_cont->max = 100;
    pid_cont->setpoint = 0;
    pid_cont->prev_err = 0;
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