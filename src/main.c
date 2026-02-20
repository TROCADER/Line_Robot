#include "main.h"
#include "../lib/pid/pid.h"
#include "../lib/qtr/qtr.h"
#include "pins.h"
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <stdio.h>
#include <stdlib.h>
#include <util/atomic.h>
#include <util/delay.h>

#define TIME_SLICE (256.0 / 32768.0)

static PID_t *pid_contA = NULL;
static PID_t *pid_contB = NULL;

static uint16_t sensor[QTR_SENSOR_COUNT];

static uint16_t base_speed = 100;
static uint16_t max_speed = 200;
static uint16_t min_speed = 0;

/**
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
    uint16_t left_sum = sensor[0] + sensor[1];
    uint16_t right_sum = sensor[3] + sensor[4];
    uint16_t total = left_sum + right_sum + sensor[2];
    double sensor_diff = 0.0;

    if (total > 0)
    {
        sensor_diff = ((double)left_sum - (double)right_sum) / (double)total;
    }
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
    init_tca();
    init_pidA();
    init_pidB();
    init_rtc();
    qtr_init();

    while (true)
    {
        uint16_t qtr_values[QTR_SENSOR_COUNT];
        qtr_read(qtr_values, true);

        ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
        {
            for (uint8_t i = 0; i < QTR_SENSOR_COUNT; i++)
            {
                sensor[i] = qtr_values[i];
            }
        }

        for (uint8_t i = 0; i < QTR_SENSOR_COUNT; i++)
        {
            printf("Sensor %d: %d\n", i, sensor[i]);
        }

        _delay_ms(100);
    }

    return 0;
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
}