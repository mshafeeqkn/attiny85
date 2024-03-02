#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define NUM_MODES   4
#define NUM_LED     6
#define LED_PIN0    PB0
#define LED_PIN1    PB1
#define LED_PIN2    PB3
#define MODE_SEL    PB2

unsigned char mode = 0;
static uint8_t data = 0x1;
volatile uint8_t repeat_count = 50;

// different states of each pin for each LED
static uint8_t led_map[][3] = {
    // { Positive pin, Negative pin, Tristate pin }
    { LED_PIN0, LED_PIN1, LED_PIN2 }, { LED_PIN1, LED_PIN0, LED_PIN2 },
    { LED_PIN1, LED_PIN2, LED_PIN0 }, { LED_PIN2, LED_PIN1, LED_PIN0 },
    { LED_PIN0, LED_PIN2, LED_PIN1 }, { LED_PIN2, LED_PIN0, LED_PIN1 }
};

// External interrupt vector, choose different mode on interrupt
ISR(INT0_vect) {
    mode++;
    mode %= NUM_MODES;

    switch(mode) {
        case 0:
            data = 0x01;
            break;
        case 1:
            data = 0xAA;
            break;
        case 2:
            data = 0xC;
            break;
        case 3:
            data = 0;
            break;
    }
}

// ADC conversion complete interrupt. used for speed control
ISR(ADC_vect) {
    repeat_count = ADCH / 3;
    ADCSRA &= ~(1 << ADIF);
}

// Glow LED in a particular index
void glow_led(uint8_t idx) {
    DDRB |= ( ( 1 << led_map[idx][0]) | (1 << led_map[idx][1]) );
    DDRB &= ~( 1 << led_map[idx][2]);
    PORTB |= ( 1 << led_map[idx][0]);
    PORTB &= ~( (1 << led_map[idx][1]) | (1 << led_map[idx][2]) );
}

// Glow LEDs if corresponding bit in the data is set
void glow_pattern(uint8_t data) {
    unsigned int mask;
    unsigned char i, r;

    // repeat_count will be determined based on ADC output
    for(r = 0; r < repeat_count; r++) {
        mask = 1;
        for(i = 0; i < NUM_LED; i++) {
            if(mask & data) {
                glow_led(i);
            }
            _delay_ms(1);
            mask <<= 1;
        }
    }
}

// Get the next bit map to be used for glowing the pattern
void next() {
    unsigned char lsb;
    unsigned char msb;

    switch(mode) {
        case 0:
            data <<= 1;
            if(0b1000000 & data) {
                data = 1;
            }
            break;

        case 1:
            data <<= 1;
            if(0 == (data & 0x3)) {
                data |= 0x1;
            }
            break;

        case 2:
            lsb = (data & 0x7);
            msb = (data >> 3);
            msb <<= 1;
            if(8 == msb) {
                msb = 0x1;
            }

            lsb >>= 1;
            if(0 == lsb) {
                lsb = 0x4;
            }

            data = (msb << 3) | lsb;
            break;

        case 3:
            data <<= 1;
            if(data & 0x40) {
                data &= ~(1);
            } else {
                data |= 1;
            }
            break;
    }
}

int main(void) {
    // Enable ADC
    ADCSRA |= ( (1 << ADEN) | (1 << ADIE) );   // Enable ADC, and ADC interrupt
    ADMUX |= (1 << ADLAR);                     // Align left

    // Enable external interrupt
    DDRB &= ~(1 << MODE_SEL);                  // set interrupt pin as input
    MCUCR |= (1 << ISC01);                     // Interrupt on falling edge
    GIMSK |= (1 << INT0);                      // Enable external interrupt

    sei();                                     // Enable global interrupt

    while(1) {
        ADCSRA |= (1 << ADSC);                 // Start ADC conversion 
        glow_pattern(data);
        next();
    }
}
