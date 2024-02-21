#include <avr/io.h>
#include <util/delay.h>

#define SWITCH_PIN      (1 << PB0)
#define LED_PIN_ON      (1 << PB4)

int main(void) {
    DDRB = 0x3E;

    while(1) {
        if( PINB & SWITCH_PIN ) {
            PORTB |= LED_PIN_ON;
            _delay_ms(10);
        } else {
            PORTB &= ~LED_PIN_ON;
            _delay_ms(10);
        }
    }

    return 0;
}
