#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>


/**
 * PB0 -> LED -> GND
 * PB2 -> Resistor-+->switch->GND
 *                 |
 *                PB2
 */

volatile uint8_t sw_press = 0;

/*
 * Interrupt service routine for external interrupt
 */

ISR(INT0_vect) {
    sw_press = 1;
}

int main(void) {
    DDRB &= ~(1 << PB2);        // PB2 - source of external interrupt
    DDRB |= (1 << PB0);         // PB0 - LED output
    PORTB = 0x0F;               // Set PB2 as push pull and PB0 as high

    GIMSK |= (1 << INT0);                   // Enable external interrupt
    MCUCR |= (1 << ISC01) | (1 << ISC00);   // Trigger raising edge of input
    sei();                                  // Enable global interrupt

    while(1) {
        if(sw_press) {
            sw_press = 0;
            PORTB ^= 0x01;
        }
    }

    return 0;
}
