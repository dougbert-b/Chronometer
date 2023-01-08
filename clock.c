/*
 * AVR clock: Drive a watch crystal and a clock movement.
 *
 * Connect:
 *   - power (-) to GND (pins 8, 22)
 *   - power (+) to VCC and AVCC (pins 7, 20)
 *   - watch XTAL to TOSC1/TOSC2 (pins 9, 10)
 *   - clock motor to PD0/PD1 (pins 2, 3)
 *   - LED + resistor to PC5 (pin 28) and GND
 *
 * Alternatively, #define PPS and get a 1PPS signal from PD0 (pin 2).
 *
 * TC2 prescaler is set to 1024, then TC2 will overflow once every
 * (roughly) 8 seconds and trigger an interrupt.
 *
 * If PPS is #defined, then the TC2 prescaler is set to 128 and TC2 will
 * overflow once per second.
 */
 
/* Output 1PPS on pin 2. */
/* #define PPS */
 
#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#define F_CPU 1000000UL  /* 1 MHz */
#include <util/delay.h>
 
#ifdef PPS
#   define MICROSECONDS_PER_INTERRUPT  1000000
#   define MICROSECONDS_PER_PULSE      1000000
#else
/* Nominally 8e6, but quartz was calibrated to be 6.5 ppm fast. */
#   define MICROSECONDS_PER_INTERRUPT  7999948
#   define MICROSECONDS_PER_PULSE     24000000
#endif
 
/*
 * On 3 V supply, seems to work reliably for pulse lengths between
 * 52 and 76 ms, or >= 100 ms
 */
#define PULSE_LENGTH 70  /* ms */
 
/* Interrupt triggered once per second. */
ISR(TIMER2_OVF_vect)
{
    static long unaccounted_microseconds;
    static uint8_t pin_bit = _BV(PD0);
#ifdef PPS
    static const uint8_t pin_mask = 0;
#else
    static const uint8_t pin_mask = _BV(PD0) | _BV(PD1);
#endif
 
    /* Send pulses at the correct average frequency. */
    unaccounted_microseconds += MICROSECONDS_PER_INTERRUPT;
    if (unaccounted_microseconds < MICROSECONDS_PER_PULSE) return;
    unaccounted_microseconds -= MICROSECONDS_PER_PULSE;
 
    /* Set the OCR2AUB flag in ASSR. */
    OCR2A = 0;
 
    /* Pulse motor. */
    PORTD = pin_bit;
    _delay_ms(PULSE_LENGTH);
    PORTD = 0;
    pin_bit ^= pin_mask;  /* next time use the other pin  */
 
    /* Wait end of TOSC1 cycle (30.5 us). */
    while (ASSR & _BV(OCR2AUB)) {/* wait */}
}
 
int main(void)
{
    uint8_t count;
 
    /* Configure PC5, PD0 and PD1 as output. */
    DDRC = _BV(DDC5);
    DDRD = _BV(DDD0) | _BV(DDD1);
 
    /* Blink while waiting for the crystal to stabilize. */
    for (count = 0; count < 3; count++) {
        _delay_ms(167);
        PORTC = _BV(PC5);
        _delay_ms(167);
        PORTC = 0;
    }
 
    /* Save power by sleeping. */
    set_sleep_mode(SLEEP_MODE_PWR_SAVE);
 
    /* Configure Timer/Counter 2. */
    ASSR |= _BV(AS2);               /* asynchronous */
    TCCR2A = 0;                     /* normal counting mode */
#ifdef PPS
    TCCR2B = _BV(CS22) | _BV(CS20); /* prescaler = 128 */
#else
    TCCR2B = _BV(CS22) | _BV(CS21) | _BV(CS20); /* prescaler = 1024 */
#endif
    TCNT2 = 1;                      /* initial value */
    while (ASSR & _BV(TCN2UB)) {/* wait */}
    TIMSK2 = _BV(TOIE2);        /* Timer Overflow Interrupt Enable */
    sei();
 
    for (;;) sleep_mode();
}

