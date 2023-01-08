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
 
 
#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/power.h>
#define F_CPU 1000000UL  // 1 MHz
#include <util/delay.h>
 

// A time unit is 10nS, or 0.01 uS.
// Calculations with time units are done with integer math.
constexpr long TIME_UNITS_PER_SECOND = 100000000;

// 1 part per million = 100 time units
constexpr long PPM = 100;

// The 32.768 kHz oscillator is prescaled by 256 before
// driving counter2.  So counter2 (nominally) increments its
// count 128 times per second.
// (One "count" is 781250 time units, according to my calculations.)
constexpr long TIME_UNITS_PER_COUNT = 100000000/128;

// Oscillator error in parts-per-million
// Positive = fast
// Adjust this empirically.
// BTW, the adjustment algorithm will fail if the error is
// greater than about 7800 PPM.
constexpr float XTAL_ERROR = +0.925;   // PPM

// The per-second time-unit surplus or defcit, due to oscillator error.
constexpr long TIME_UNIT_ERROR_PER_SECOND = (long)(XTAL_ERROR * PPM);

// Doug: Model 12888 clock movement has 31 mS pulse length,
// with 1.4 volt square wave.  Coil resistance: 265 ohms.
#define PULSE_LENGTH 25  // ms
 

// Interrupt handler, usually triggered once per second (i.e. every 32768
// oscillator cycles).

ISR(TIMER2_COMPA_vect)
{
    // I/O pins connected to clock motor.
    constexpr uint8_t PIN_MASK(_BV(PD0) | _BV(PD1));

    // Keeps track of the pulse polarity.
    static uint8_t pin_bit(_BV(PD0));
 
    // Keep track of the accumulated error
    static long accumulated_time_unit_error(0);

    // Pulse motor.
    PORTD = pin_bit;
    _delay_ms(PULSE_LENGTH);
    PORTD = 0;
    pin_bit ^= PIN_MASK;  // next time use the other pin 

    // TODO: tweak the error based on power suply voltage.
    accumulated_time_unit_error += TIME_UNIT_ERROR_PER_SECOND;

    // If the error has gotten too large (i.e. clock is fast), we need to add
    // one extra count in the next cycle.  Likewise, if the error has
    // gotten too negative (i.e. clock is slow), we have to take
    // away one count in the next cycle.
    //
    // An example:
    // For a 1 PPM error, a 1/128 second correction will be made about every 2 hours.
 
    bool ocr2_touched = false;

    if (accumulated_time_unit_error >= TIME_UNITS_PER_COUNT) {
      accumulated_time_unit_error -= TIME_UNITS_PER_COUNT;
      OCR2A = 128;  // Next clock tick will be one count later than usual.
      ocr2_touched = true;
    } else if (accumulated_time_unit_error <= -TIME_UNITS_PER_COUNT) {
      accumulated_time_unit_error += TIME_UNITS_PER_COUNT;
      OCR2A = 126; // Next clock tick will be one count earlier than usual.
      ocr2_touched = true;
    } else if (OCR2A != 127) {
      // An adjustment was done during the previous interrupt; now
      // we must re-establish the default value.
      OCR2A = 127;  // Default value: exactly 32678 oscillator pulses.
      ocr2_touched = true;
    }

    // If OCR2 was modified, wait until the update completes.
    if (ocr2_touched) {
      while (ASSR & _BV(OCR2AUB)) {}
    }
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
    ASSR = _BV(EXCLK) | _BV(AS2);  // External clock signal (no xtal) and asynchronous

    TCCR2A = _BV(WGM21);           // CTC mode: clear counter on match with OCR2A

    TCCR2B = _BV(CS22) | _BV(CS21); // prescaler = 256

    OCR2A = 127;		    // Default counting range: 0 thru 127 (inclusive)

    // Wait until OCR2A, TCCR2A, and TCCR2B updates complete.
    while (ASSR & (_BV(OCR2AUB) | _BV(TCR2AUB) | _BV(TCR2BUB))) {}

    TIMSK2 = _BV(OCIE2A);           // Timer2 compare match A interrupt enable
    sei();
 
    for (;;) sleep_mode();     // All further processing is done in the interrupt handler.
}

