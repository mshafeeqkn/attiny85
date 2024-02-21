#include <avr/io.h>
#include <util/delay.h>

#define SWITCH_PIN      (1 << PB0)
#define LED_PIN_ON      (1 << PB4)

int main(void) {
    DDRB |= LED_PIN_ON;                     // Set LED pin as output
    DDRB &= ~SWITCH_PIN;                    // Set Switch pin as input

    PORTB |= SWITCH_PIN;                    // Enable pull up resistor at LED pin

    while(1) {
        if( PINB & SWITCH_PIN ) {           // If the switch pin is HIGH
            PORTB |= LED_PIN_ON;
            _delay_ms(10);                  // switch debouncing delay
        } else {
            PORTB &= ~LED_PIN_ON;
            _delay_ms(10);
        }
    }

    return 0;
}
