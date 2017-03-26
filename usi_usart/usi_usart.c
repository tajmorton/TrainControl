#include <avr/io.h>
#include <avr/interrupt.h>

#define F_CPU 1000000  // 1 MHz
#define BAUD 4800
#define CYCLES_PER_BIT (F_CPU/BAUD)

#if (CYCLES_PER_BIT > 255)
// have to prescale TC0 because counter will overflow
#define DIVISOR 8
#define PRESCALE 2 // CLK[io]/8
#else
#define DIVISOR 1
#define PRESCALE 1  // CLK[io]/1
#endif

#define FULL_BIT_TICKS ((CYCLES_PER_BIT/DIVISOR) + 10)
#define HALF_BIT_TICKS (FULL_BIT_TICKS/2)

#define SETUP_TIME (62) // number of cycles required to setup for read after seeing start bit
#define TIMER_START_DELAY (SETUP_TIME/DIVISOR) // value to count to while waiting for the above

#define PIN(x) (1 << x)
#define PCINT12 4

#include <util/delay.h>
volatile uint8_t serialDataReady = 0;
volatile uint8_t serialInput;

void setupRead(void);
void sawStartBit(void);

uint8_t ReverseByte (uint8_t x) {
    x = ((x >> 1) & 0x55) | ((x << 1) & 0xaa);
    x = ((x >> 2) & 0x33) | ((x << 2) & 0xcc);
    x = ((x >> 4) & 0x0f) | ((x << 4) & 0xf0);
    return x;
}

ISR(BADISR_vect) {
    PORTB &= ~PIN(0);
}

void setupRead(void) {
    /*
     * Configures USI to perform a read.
     */
    DDRB &= ~PIN(4); // PB4 as input
    PORTB |= PIN(4); // pullup on PB4
    USICR = 0; // disable USI before we see the falling edge of the start bit

    GIMSK |= 1<<PCIE1; // enable PCIE1 interrupts, so we can get interrupt on PCINT12/PB4
    PCMSK1 |= 1<<PCINT12; // enable interrupt from PCINT12/PB4
}

ISR(PCINT1_vect) {
    /*
     * ISR that is triggered by the start bit on PB4.
     */
    if (!(PINB & PIN(4))) {
        // falling edge on PB4
        PORTB |= PIN(1);
        sawStartBit();
    }
}

void sawStartBit(void) {
    /*
     * Configures timer and waits until the middle of the start bit to fire
     * first interrupt from timer (so we can enable the USI).
     */
    GIMSK &= ~(1<<PCIE1); // no longer trigger on changing PB4, now that we've seen the start bit

    // configure timer
    TCCR0A |= 2<<WGM00; // put TC0 in Clear Timer on Compare mode
    TCCR0B = PRESCALE<<CS10; // clock select/prescale value

    GTCCR |= 1<<PSR10; // reset prescaler
    TCNT0 = 0; // initialize timer

    // Configured at 4800 baud @ 1 MHz, we are at approximately halfway
    // through the start bit by the time this line is hit.
    // Start the FULL_BIT_TICKS so that the first time the timer triggers,
    // we are ~halfway through the first bit to be read.
    OCR0A = FULL_BIT_TICKS;

    // turn on output compare interrupt, this is just so we can
    // toggle an output pin and see how accurately we're hitting the
    // middle of the bit
    TIFR0 = 1<<OCF0A; // clear output compare interrupt on OCR0A
    TIMSK0 |= 1<<OCIE0A; // enable interrupt on compare of OCR0A
    PORTB &= ~PIN(1);

    // configure USI hardware
    USICR = (
        1<<USICS0 | // clock in on TC0 compare match
        0<<USIWM0 | // all hardware wire modes disabled
        1<<USIOIE   // generate interrupt when reading a bit (so we can count to 8)
    );

    USISR = (
        1<<USIOIF | // clear USI overflow interrupt flag
        1<<USISIF | // clear read-bit flag
        8<<USICNT0  // generate USI overflow interrupt after reading 8 bits
    );
}

ISR(TIM0_COMPA_vect) {
    /*
     * Interrupt fired once we are at the middle of the start bit.
     * Should be used to configure timer to read once per bit and
     * to enable the USI.
     * Currently just toggles a bit so we can measure how accurately
     * we're hitting the middle of the bit.
     */
    PORTB ^= PIN(1);
}

ISR(USI_OVF_vect) {
    /*
     * Interrupt fired once we have read 8 bits.
     */

    TCCR0B = 0;  // turn off timer
    serialInput = USIDR; // read input data
    serialDataReady = 1;

    PORTB &= ~PIN(1);

    USICR = 0; // disable USI now that we've read the whole byte in

    // clear interrupt flag on PCIE1 that it will have triggered while
    // reading in data (to prevent it from triggering an interrupt
    // when we re-enable it on the next line)
    GIFR = 1<<PCIF1;

    // re-enable pin change interrupts so we can read the next byte
    // (when it arrives)
    GIMSK |= 1<<PCIE1;
}

int main(void) {
    uint8_t serData = 0;

    DDRB |= PIN(0) | PIN(1);
    PORTB = 1;
    setupRead();
    sei();

    uint8_t i = 0;

    while(1) {
        if (serialDataReady) {
            serData = ReverseByte(serialInput);
            serialDataReady = 0;

            if (serData == 0xAA) {
                PORTB |= PIN(0);
            }
            else if (serData == 0x55) {
                PORTB &= ~PIN(0);
            }
            else {
                PORTB |= PIN(0);
                _delay_ms(200);
                PORTB &= ~PIN(0);
                _delay_ms(200);

                PORTB |= PIN(0);
                _delay_ms(200);
                PORTB &= ~PIN(0);
                _delay_ms(200);
                PORTB &= ~PIN(1);
            }
        }
    }
}
