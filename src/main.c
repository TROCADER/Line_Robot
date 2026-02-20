#include "main.h"
#include "../lib/echo/echo.h"
#include "../lib/pid/pid.h"
#include "../lib/qtr/qtr.h"
#include "pins.h"
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <util/atomic.h>
#include <util/delay.h>

#define RTC_PIT_CYCLES 256.0
#define RTC_CLOCK_HZ 32768.0
#define PID_DT_MS ((RTC_PIT_CYCLES * 1000.0) / RTC_CLOCK_HZ)
#define LINE_POSITION_CENTER 2000
#define WHITE_LEVEL_THRESHOLD 2950
#define MOTOR_TURN_SPEED 180
#define LINE_MIN_STRENGTH 30

static PID_t *pid_cont = NULL;

static volatile uint16_t sensor[QTR_SENSOR_COUNT];
static volatile uint16_t echo;

static int16_t base_speed = 100;
static int16_t max_speed = 200;
static int16_t min_speed = 0;
static volatile int16_t previous_error = 0;

/**
 * prepare the re-routing of stdout to UART2
 */
static int uart_putchar(char c, FILE *stream);
static FILE mystdout = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);

static bool all_sensors_white(const uint16_t values[]);
static int16_t compute_line_error(const uint16_t values[]);
static int16_t clamp_speed(int16_t speed);
static void motor_drive(int16_t left_speed, int16_t right_speed);

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
    uint16_t sensor_snapshot[QTR_SENSOR_COUNT];
    int16_t left_speed;
    int16_t right_speed;

    for (uint8_t i = 0; i < QTR_SENSOR_COUNT; i++)
    {
        sensor_snapshot[i] = sensor[i];
    }

    if (all_sensors_white(sensor_snapshot))
    {
        if (previous_error > 0)
        {
            left_speed = min_speed;
            right_speed = MOTOR_TURN_SPEED;
        }
        else
        {
            left_speed = MOTOR_TURN_SPEED;
            right_speed = min_speed;
        }
    }
    else
    {
        int16_t error = compute_line_error(sensor_snapshot);
        double correction = pid_calc(pid_cont, (double)error, PID_DT_MS, true);

        previous_error = error;

        left_speed = base_speed - correction;
        right_speed = base_speed + correction;
    }

    motor_drive(left_speed, right_speed);

    RTC.PITINTFLAGS = RTC_PI_bm; // Clear interrupt flag
}

int main(void)
{
    UARTInit(19200);
    sei();

    init_pins();
    init_tca();
    init_pid();
    init_rtc();
    qtr_init();
    echo_init();

    while (true)
    {
        uint16_t qtr_values[QTR_SENSOR_COUNT];
        uint16_t distance_cm;

        qtr_read(qtr_values, true);
        echo = echo_read();
        distance_cm = echo_to_cm(echo);

        ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
        {
            for (uint8_t i = 0; i < QTR_SENSOR_COUNT; i++)
            {
                sensor[i] = qtr_values[i];
            }
        }

        printf("S: %u %u %u %u %u | ECHO(us): %u | DIST(cm): %u\n", sensor[0], sensor[1], sensor[2], sensor[3],
               sensor[4], echo, distance_cm);

        _delay_ms(25);
    }

    return 0;
}

void init_pid()
{
    pid_cont = calloc(1, sizeof(PID_t));
    pid_cont->Kp = 0.08;
    pid_cont->Ki = 0.0;
    pid_cont->Kd = 0.45;
    pid_cont->integ = 0;
    pid_cont->min = -(double)max_speed;
    pid_cont->max = (double)max_speed;
    pid_cont->setpoint = 0;
    pid_cont->prev_err = 0;
}

void init_rtc()
{
    RTC.CTRLA = RTC_PRESCALER_DIV1_gc;
    RTC.CLKSEL = CLKSEL_OSC32K_gc; // 32768.0
    RTC.PITCTRLA = RTC_PITEN_bm | RTC_PERIOD_CYC256_gc;
    RTC.PITINTCTRL = RTC_PI_bm;
}

void init_tca()
{
    TCA0.SINGLE.CTRLA = 0;
    TCA0.SINGLE.CTRLB = TCA_SINGLE_CMP0EN_bm | TCA_SINGLE_CMP1EN_bm | TCA_SINGLE_WGMODE_SINGLESLOPE_gc;
    TCA0.SINGLE.PER = 255;
    TCA0.SINGLE.CMP0BUF = 0;
    TCA0.SINGLE.CMP1BUF = 0;
    PORTMUX.TCAROUTEA = PORTMUX_TCA0_PORTA_gc;
    TCA0.SINGLE.CTRLA = TCA_SINGLE_ENABLE_bm | TCA_SINGLE_CLKSEL_DIV64_gc;
}

void init_pins()
{
    // Initialize all pins
    // TCA0 WO0/WO1 on PA0/PA1
    PORTA.DIRSET = PIN0_bm | PIN1_bm;
}

bool all_sensors_white(const uint16_t values[])
{
    for (uint8_t i = 0; i < QTR_SENSOR_COUNT; i++)
    {
        if (values[i] < WHITE_LEVEL_THRESHOLD)
        {
            return false;
        }
    }

    return true;
}

int16_t compute_line_error(const uint16_t values[])
{
    uint32_t weighted_sum = 0;
    uint32_t strength_sum = 0;

    for (uint8_t i = 0; i < QTR_SENSOR_COUNT; i++)
    {
        uint16_t level = values[i];
        uint16_t strength = (level >= QTR_MAX_TIME) ? 0 : (QTR_MAX_TIME - level);

        if (strength < LINE_MIN_STRENGTH)
        {
            strength = 0;
        }

        strength_sum += strength;
        weighted_sum += strength * (i * 1000U);
    }

    if (strength_sum == 0)
    {
        return previous_error;
    }

    int16_t position = (int16_t)(weighted_sum / strength_sum);
    return (int16_t)(LINE_POSITION_CENTER - position);
}

int16_t clamp_speed(int16_t speed)
{
    if (speed > max_speed)
    {
        return max_speed;
    }
    if (speed < min_speed)
    {
        return min_speed;
    }
    return speed;
}

void motor_drive(int16_t left_speed, int16_t right_speed)
{
    float scale = echo_power_scale(echo);
    left_speed = (int)clamp_speed(left_speed) * scale;
    right_speed = (int)clamp_speed(right_speed) * scale;

    TCA0.SINGLE.CMP0BUF = left_speed;
    TCA0.SINGLE.CMP1BUF = right_speed;
}