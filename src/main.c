#include "main.h"
#include "../lib/pid/pid.h"
#include "pins.h"
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <stdio.h>
#include <stdlib.h>
#include <util/atomic.h>
#include <util/delay.h>

#define TIME_SLICE (256.0/32768.0)

static PID_t *pid_contA = NULL;
static PID_t *pid_contB = NULL;

static uint16_t lsensor = 0;
static uint16_t rsensor = 0;
static uint16_t sensor[IR_SENSORS_NR];

static uint16_t base_speed = 100;
static uint16_t max_speed = 200;
static uint16_t min_speed = 0;

/**ande seminarium
Thursday, March 12⋅13:15 – 17:00

 * prepare the re-routing of stdout to UART2
 */
static int uart_putchar(char c, FILE *stream);
static int uart_getchar(FILE *stream);
static FILE mystdout = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);

/**
 * @param baudrate - UART baudrate
 * @brief initializes the UART module
 */
void UARTInit(uint32_t baudrate)
{
    uint16_t baud;
    baud = ((float)(F_CPU * 64 / (16 * (float)baudrate)) + 0.5);
    PORTMUX.USARTROUTEA = PORTMUX_USART2_DEFAULT_gc; // TxD PF0, RxD PF1
    PORTF.DIRSET = PIN0_bm;
    USART2.BAUD = baud;
    USART2.CTRLB = USART_TXEN_bm | USART_RXEN_bm;
    USART2.CTRLC = USART_CMODE_ASYNCHRONOUS_gc | USART_PMODE_DISABLED_gc | USART_SBMODE_1BIT_gc | USART_CHSIZE_8BIT_gc;
    USART2.CTRLA = 0;
    stdout = &mystdout;
}

/**
 * @param c - char/byte to send
 * @brief send a byte to USART2, used for stdout
 */
static int uart_putchar(char c, FILE *stream)
{
    while (!(USART2.STATUS & USART_DREIF_bm))
        ;               // wait if still transmitting
    USART2.TXDATAL = c; // next byte into the send register
    return 0;
}

ISR(RTC_PIT_vect)
{
    uint16_t sensor_tot = sensor[0] + sensor[1];
    double sensor_diff = 0.0;

    sensor_diff = (sensor[0] - sensor[1]) / sensor_tot;
    double correctionA = pid_calc(pid_contA, sensor_diff, TIME_SLICE, false);
    double correctionB = pid_calc(pid_contB, sensor_diff, TIME_SLICE, false);

    uint16_t lspeed = base_speed + correctionA;
    uint16_t rspeed = base_speed + correctionB;

    printf("Left Speed: %d\n", lspeed);
    printf("Right Speed: %d\n", rspeed);

    // TODO: Apply power to motors
    TCA0.SINGLE.CMP0 = -lspeed;
    TCA0.SINGLE.CMP1 = -rspeed;

    RTC.PITINTFLAGS = RTC_PI_bm; // Clear interrupt flag
}

int main(void)
{
    UARTInit(19200);
    sei();

    init_pins();
    init_adc();
    init_tca();
    init_pidA();
    init_pidB();
    init_rtc();

    while (true)
    {
        for (uint8_t i = 0; i < IR_SENSORS_NR; i++)
        {
            read_sensor(i, &sensor[i]);
            printf("Sensor %d: %d\n", i, sensor[i]);
            _delay_ms(100);
        }
    }

    return 0;
}

void read_sensor(uint8_t sensor, uint16_t *sensor_data)
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        ADC0.MUXPOS = sensor;
        ADC0.COMMAND = ADC_STCONV_bm;
        while (ADC0.COMMAND & ADC_SPCONV_bm)
        {
        }
        *sensor_data = ADC0.RES;
    }
}

void init_pidA()
{
    pid_contA = calloc(1, sizeof(PID_t));
    pid_contA->Kp = 50;
    pid_contA->Ki = 0;
    pid_contA->Kd = 0;
    pid_contA->integ = 0;
    pid_contA->min = -100;
    pid_contA->max = 100;
    pid_contA->setpoint = 0;
    pid_contA->prev_err = 0;
}

void init_pidB()
{
    pid_contB = calloc(1, sizeof(PID_t));
    pid_contB->Kp = 50;
    pid_contB->Ki = 0;
    pid_contB->Kd = 0;
    pid_contB->integ = 0;
    pid_contB->min = -100;
    pid_contB->max = 100;
    pid_contB->setpoint = 0;
    pid_contB->prev_err = 0;
}

void init_adc()
{
    VREF.ADC0REF = VREF_REFSEL_VDD_gc;
    PORTD.PIN0CTRL = PORT_ISC_INPUT_DISABLE_gc;
    PORTD.PIN1CTRL = PORT_ISC_INPUT_DISABLE_gc;
    // PORTD.PIN2CTRL = PORT_ISC_INPUT_DISABLE_gc;
    // PORTD.PIN3CTRL = PORT_ISC_INPUT_DISABLE_gc;

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
    RTC.PITCTRLA = RTC_PITEN_bm | RTC_PERIOD_CYC256_gc;
    RTC.PITINTCTRL = RTC_PI_bm;
}

void init_tca()
{
    TCA0.SINGLE.CTRLA = TCA_SINGLE_ENABLE_bm | TCA_SINGLE_CLKSEL_DIV64_gc;
    TCA0.SINGLE.CTRLB = TCA_SINGLE_CMP0_bm | TCA_SINGLE_CMP1_bm | TCA_SINGLE_WGMODE_DSBOTTOM_gc;
    TCA0.SINGLE.PER = 32768;
    PORTMUX.TCAROUTEA = PORTMUX_TCA0_PORTA_gc;
}

void init_pins()
{
    // Initialize all pins
    // PIN A - Motors
    PORTA.DIRSET = PIN0_bm | PIN1_bm;
    // PORTC.OUT = 0 | PIN_LEFT_MOTOR_A | PIN_LEFT_MOTOR_B | PIN_RIGHT_MOTOR_A | PIN_RIGHT_MOTOR_B;

    // PIN D - Sensors
    // PORTD.DIR = 0 | PIN_LEFT_IR_ANALOG | PIN_RIGHT_IR_ANALOG;
    // PORTD.IN = 0 | PIN_LEFT_IR_ANALOG | PIN_RIGHT_IR_ANALOG;
}