#include <avr/io.h>
#include <util/delay.h>

/**
 *  Connections
 *  PB0 --> switch --> GND
 *  PB4 --> LED ->> GND
 */


#define SWITCH_PIN      (1 << PB0)
#define LED_PIN_ON      (1 << PB4)

int main(void) {
    DDRB |= 0xFF;                     // Set LED pin as output
    while(1) {
        PORTB ^= 0xFF;
        _delay_ms(500);
/*
        if( PINB & SWITCH_PIN ) {           // If the switch pin is HIGH
            PORTB |= LED_PIN_ON;
            _delay_ms(10);                  // switch debouncing delay
        } else {
            PORTB &= ~LED_PIN_ON;
            _delay_ms(10);
        }
*/
    }

    return 0;
}
