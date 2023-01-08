//
// AVR clock: Drive a clock movement.
//
// Connect:
//   - power (-) to GND (pins 8, 22)
//   - power (+) to VCC and AVCC (pins 7, 20)
//   - 32kHz oscillator into TOSC1 (pin 9)
//   - clock motor to PD0/PD1 (pins 2, 3)
//   - LED + resistor to PC5 (pin 28) and GND
//
// If TC2 prescaler is set to 32, then TC2 will overflow once every
// (nominally) 0.25 seconds and trigger an interrupt.  That is what is done
// here.
//
//
 
 
#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/power.h>
#define F_CPU 1000000UL  // 1 MHz
#include <util/delay.h>
 

// A time unit is 10nS, or 0.01 uS.

// This value is exactly correct for the clock motor: one tick per second
constexpr long TIME_UNITS_PER_TICK = 100000000;

// This value is nominal (i.e. if the crystal is perfect): 4 interrupts per second.
constexpr long NOMINAL_TIME_UNITS_PER_INTERRUPT = 25000000;

// 1 part per million = 100 time units
constexpr long PPM = 100;

// Xtal error in parts-per-million
// Positive = fast
// Adjust this empirically
constexpr float XTAL_ERROR = +0.925;

constexpr long TIME_UNITS_PER_INTERRUPT =
	    NOMINAL_TIME_UNITS_PER_INTERRUPT -
	      (long)(XTAL_ERROR * PPM);

 
// On 3 V supply, seems to work reliably for pulse lengths between
// 52 and 76 ms, or >= 100 ms
// 
// Doug: 12888 movement has 31 mS pulse length, with 1.4 volt square wave.
// Coil resistance: 265 ohms.
#define PULSE_LENGTH 25  // ms
 

// Interrupt handler, triggered nominally 4 times per second.
ISR(TIMER2_OVF_vect)
{
    static long unaccounted_time_units;
    static uint8_t pin_bit = _BV(PD0);
    static constexpr uint8_t PIN_MASK = _BV(PD0) | _BV(PD1);
 
    // Send pulses at the correct average frequency.
    unaccounted_time_units += TIME_UNITS_PER_INTERRUPT;
    if (unaccounted_time_units < TIME_UNITS_PER_TICK) return;

    // We're due for a tick
    unaccounted_time_units -= TIME_UNITS_PER_TICK;
 
    // Pulse motor.
    PORTD = pin_bit;
    _delay_ms(PULSE_LENGTH);
    PORTD = 0;
    pin_bit ^= PIN_MASK;  // next time use the other pin 
 
}
 

int main(void)
{
    // Configure PC5, PD0 and PD1 as output.
    DDRC = _BV(DDC5);
    DDRD = _BV(DDD0) | _BV(DDD1);

    // Set all port B pins to inputs.
    DDRB = 0x00;

    // Turn on pullups of unused input pins to
    // prevent float.  Saves power.
    PORTB = 0xff; // All port B pins pullups are on.

    // Disable unused peripherals.
    power_spi_disable();
    power_timer0_disable();
    power_timer1_disable();
    power_twi_disable(); 

    // Blink while waiting for the crystal to stabilize.
    for (uint8_t count = 0; count < 3; count++) {
        _delay_ms(167);
        PORTC = _BV(PC5);
        _delay_ms(167);
        PORTC = 0;
    }
 
    // Save power by sleeping.
    set_sleep_mode(SLEEP_MODE_PWR_SAVE);
 
    // Configure Timer/Counter 2.
    ASSR |= _BV(EXCLK);             // external clock signal - no xtal
    ASSR |= _BV(AS2);               // asynchronous
    TCCR2A = 0;                     // normal counting mode
    TCCR2B = _BV(CS21) | _BV(CS20); // prescaler = 32
    TCNT2 = 1;                      // initial value
    while (ASSR & _BV(TCN2UB)) ;    // wait until TCNT2 update completes.
    TIMSK2 = _BV(TOIE2);            // Timer Overflow Interrupt Enable
    sei();
 
    for (;;) sleep_mode();
}

