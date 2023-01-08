//
// $Revision: 1.14 $
//
//
//
// AVR-based chronometer: Drives an analog clock movement.
// Includes voltage compensation and low-voltage warning.
// Doug Braun 10/29/2019
//
// Written for an ATmega48PA, and AVR-GCC
//
// Recommended compiler options:
// CFLAGS=-mmcu=atmega48pa -Os -Wall -Wextra -std=c++11 -Wl,-u,vfprintf -lprintf_min
//
// Pin connections (for 28-pin DIP package):
//   - power (-) to GND (pins 8, 22)
//   - power (+) to VCC and AVCC (pins 7, 20)
//   - 32kHz oscillator into TOSC1 (pin 9)
//   - clock motor to PD2/PD3 (pins 4, 5)
//   - Pulldown adjustment switches to inputs PC4 and PC5 (pins 27, 28)
//   - MISO, MOSI, GND, RESET, and VCC go to programming connector.
//   - Pins 2,3 available as serial port for debugging.
//
 
 
#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/power.h>
#define F_CPU 1000000UL  // 1 MHz
#include <util/delay.h>
 
#include <stdio.h>


// Calibration Parameters
//
//
// Oscillator error in parts-per-million at nominal voltage: approx 3 volts.
// Positive = fast.  Adjust this empirically.
// If the clock is running slow, decrease the value.
// If it is running fast, increase it.
// One second per year equals 0.03171 parts-per-million.
// BTW, the adjustment algorithm will fail if the error is greater than about 7800 PPM.
constexpr float XTAL_ERROR_VNOM = +1.2122;  // (was +1.1482)   // in PPM

// Nominal voltage (typical voltage of fresh batteries)
constexpr float VNOM = 3.15;  // Volts

// Correction factor for non-nominal voltages.
// The oscillator typically speeds up as the voltage is increased.
constexpr float VOLTAGE_COEFF = +1.732;   // PPM per Volt: measured empirically

// The threshold for a low-voltage warning indication
constexpr float LOW_VOLTAGE = 2.70;  // Based on DS32Khz datasheet

// Actual ADC voltage reference (Vbg), as measured.
constexpr float VREF = 1.110;



// 1 part per million = 100 time units
constexpr long PPM = 100;

// A time unit is 10nS, or 0.01 uS.
// Calculations with time units are done with integer math.
constexpr long TIME_UNITS_PER_SECOND = 100000000;

// The 32.768 kHz oscillator is prescaled by 256 before
// driving counter2.  So counter2 (nominally) increments its
// count 128 times per second.
// (One "count" is 781250 time units, according to my calculations.)
constexpr long TIME_UNITS_PER_COUNT = TIME_UNITS_PER_SECOND/128;


#if 0
// This maps the ADC output (an integer from 0 to 1023) to the corresponding
// supply voltage, in volts.
// This has costly division and float math.
constexpr float adc_to_volts(long adc_val)
{
  return (adc_val > 0) ? (1024.0 * VREF / (float)adc_val) : 0.0;
}
#endif

// This maps the ADC output (an integer from 0 to 1023) to the corresponding
// supply voltage, in integeral millivolts. This uses no costly float math.
constexpr long adc_to_millivolts(long adc_val)
{
  return (adc_val > 0) ? (1000 * ((long)(1024.0 * VREF)) / adc_val) : 0.0;
}

// The inverse of the above function.
// This has floats and division, but it is constexpr,
// so there is no overhead if the voltage value is constexpr, too.
constexpr long voltage_to_adc(float voltage)
{
  return (long)(1024.0 * VREF / voltage);
}


// This tests if the given ADC output represents a low (brownout) voltage.
// This has no costly runtime division or float math.
constexpr bool is_low_voltage(long adc_val)
{
  // As the ADC value increases, the voltage decreases.
  return adc_val > voltage_to_adc(LOW_VOLTAGE);
}


// This maps the ADC output (an integer from 0 to 1023) to the corresponding
// correction factor, in integer time_units.  This does no runtime float math,
// and only one integer addition and one division.
long calculate_correction(long adc_val)
{
  // In float math:
  //   voltage = (1024.0 * VREF / adc_val);
  //   result = PPM * ((voltage*VOLTAGE_COEFF) + (XTAL_ERROR_VNOM - (VNOM*VOLTAGE_COEFF)))
  // or
  //   result = ((PPM*1024*VREF*VOLTAGE_COEFF)/adc_val) +
  //                        (PPM*(XTAL_ERROR_VNOM-(VNOM*VOLTAGE_COEFF)))

  // approx 100*1024*1.11*1.7 = 193229
  constexpr long a = (long)(PPM*1024*VREF*VOLTAGE_COEFF);

  // approx (100*(0.925-(3.15*1.7))) = -443
  constexpr long b = (long)(PPM*(XTAL_ERROR_VNOM-(VNOM*VOLTAGE_COEFF)));

  // adc_val is roughly in the range of 300 - 500

  return a / adc_val + b;
}


// Interfaces directly with the UART registers.
// After calling this, wait a couple of milliseconds
// before putting the processor to sleep.
int uart_putchar(char c, FILE *stream) {
  if (c == '\n') {
    uart_putchar('\r', stream);
  }
  loop_until_bit_is_set(UCSR0A, UDRE0);
  UDR0 = c;

  return 0;
}

FILE uart_output;

// Set the UART speed to 9600 baud.
void baud9600()
{
#define BAUD 9600
#include <util/setbaud.h>
  UBRR0H = UBRRH_VALUE;
  UBRR0L = UBRRL_VALUE;
#if USE_2X
  UCSR0A |= _BV(U2X0);
#else
  UCSR0A &= ~(_BV(U2X0));
#endif
}


void uart_init() {
  power_usart0_enable();

  baud9600();

  UCSR0B = _BV(TXEN0);   /* Enable TX only */
  UCSR0C = _BV(UCSZ01) | _BV(UCSZ00) | _BV(USBS0); /* 8-bit data, 2 stop bits */

  _delay_ms(10); // Wait for line to settle

  fdev_setup_stream(&uart_output, uart_putchar, NULL, _FDEV_SETUP_WRITE);
  stdout = &uart_output;
}

void uart_shutdown() {
  fflush(&uart_output);
  fdev_close();
  power_usart0_disable();
}


// Return the ADC output, which tells us the power supply voltage.
long read_adc()
{
  power_adc_enable();
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  _delay_ms(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADEN); // Enable ADC
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)) {}
  long result = ADCL;
  result |= ADCH<<8;
  power_adc_disable();
  return result;
}


// Doug: Model 12888 clock movement has 31 mS pulse length,
// with 1.4 volt square wave.  Coil resistance: 265 ohms.
 
void pulse_motor()
{
  // I/O pins connected to clock motor.
  constexpr uint8_t PIN_MASK(_BV(PD2) | _BV(PD3));

  // Keeps track of the pulse polarity.
  static uint8_t pin_bit{_BV(PD2)};

  // Pulse motor.

  // Use a 50% PWM waveform, since we are driving the motor with 3 volts
  // instead of 1.5 volts.
  // TODO: Tristate motor pins when not driving, to better handle
  // induced current?
  // TODO: Adjust duty cycle based on battery voltage, to provide constant
  // motor power.

  for (uint8_t count = 0; count < 16; count++) {
    PORTD |= pin_bit;
    _delay_ms(1);
    PORTD &= ~pin_bit;
    _delay_ms(1);
  }

  pin_bit ^= PIN_MASK;  // next time use the other pin 
}


// Interrupt handler, usually triggered once per second (i.e. every 32768
// oscillator cycles).

ISR(TIMER2_COMPA_vect)
{

  // Keep track of the accumulated error
  static long accumulated_time_unit_error{0};
  static bool skipped_tick{false};

  // Test "stop" button
  if (bit_is_clear(PINC, PC4)) {
    // If the "stop" button is pressed, freeze until it is released,
    // setting up the counter to generate the next interrupt in
    // about 0.8 seconds.

    _delay_ms(10);  // Debounce

    accumulated_time_unit_error = 0;
    skipped_tick = false;

    while (bit_is_clear(PINC,PC4)) {
      TCNT2 = 25;
      loop_until_bit_is_clear(ASSR, TCN2UB); // wait for update to finish
    }
    return;
  }

  // If the "fast" button is pressed, do a double pulse.
  // (and skip the voltage check).
  if (bit_is_clear(PINC, PC5)) {
    accumulated_time_unit_error = 0;
    skipped_tick = false;
    pulse_motor();
    _delay_ms(330);
    pulse_motor();
    return;
  }

  // Normal case: no buttons pressed.
  
  // To save power, don't read the ADC and update the correction
  // every second.

  static long adc_val{0};
  static long correction{0};
  static bool low_voltage{false};
  static int adc_read_cnt{0};

  if (adc_read_cnt > 0) {
    --adc_read_cnt;
  } else {
    // Read the ADC, which is measuring the supply voltage.
    adc_val = read_adc();
    // Update the correction value
    correction = calculate_correction(adc_val);
    low_voltage = is_low_voltage(adc_val);

    // Normally, check the voltage only once a minute.
    // But if it is already low, check it frequently, so
    // that we can tuirn off the low-voltage warning
    // as soon as possible after the proper vltage is
    // restored (e.g. by replacing the batteries).
    adc_read_cnt = low_voltage ? 5 : 60;
  }

  if (skipped_tick) {
    // Do a double tick to make up for the previous skipped tick
    pulse_motor();
    _delay_ms(250);
    pulse_motor();
    skipped_tick = false;
  } else if (is_low_voltage(adc_val)) {
    // If the voltage is too low, skip this tick, and remember that we did it.
    skipped_tick = true;
  } else {
    // Default situation: just pulse motor
    pulse_motor();
  }


  accumulated_time_unit_error += correction;

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
    loop_until_bit_is_clear(ASSR, OCR2AUB);
  }

  // Print only if UART was set up.
  if (stdout) {
    static unsigned long count{0};
    printf("%lu : %ld mV correction %ld  accum error %ld/%ld\n",
	   count, adc_to_millivolts(adc_val), correction,
	   accumulated_time_unit_error, TIME_UNITS_PER_COUNT);
    fflush(stdout);
    _delay_ms(10);  // Allow last character to be sent out
    count++;
  }
}
 

int main()
{
  // Configure PD2 and PD3 as output to motor, and PD1 as serial output for
  // debugging.
  DDRD = _BV(DDD1) | _BV(DDD2) | _BV(DDD3);

  // Turn on pullups on PC0 to PC5.
  // Only PC4 and PC5 are actually used.
  PORTC = 0x3f;


  // Set all port B and C pins to inputs.
  DDRB = 0x00;
  DDRC = 0x00;

  // Turn on pullups of unused input pins to
  // prevent float.  Saves power.
  PORTB = 0xff; // All port B pins pullups are on.
  PORTD = 0xff;

  // Disable unused peripherals.
  power_spi_disable();
  power_timer0_disable();
  power_timer1_disable();
  power_twi_disable();
  power_usart0_disable();
  power_adc_disable();

  // do this only if debugging output is desired
  // uart_init();

  // Wait one second for the external oscillator to stabilize.
  _delay_ms(1000);

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


