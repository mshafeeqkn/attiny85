#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

ISR(TIMER0_COMPA_vect) {
    PORTB ^= (1 << PB4);
}

void init_timer() {
    /* Timer/Counter Conrol Register A */
    TCCR0A |= (0 << COM0A1) | (0 << COM0A0) |
              (0 << COM0B1) | (0 << COM0B0) |
              (1 << WGM01)  | (0 << WGM00);     //CTC Mode

    /* Timer/Counter Control Register B */
    TCCR0B |= (0 << FOC0A) | (0 << FOC0B) |
              (0 << WGM02) |
              (1 << CS02)  | (1 << CS01) | (0 << CS00); // T0 falling edge clock

    TCNT0 = 0x00;
    OCR0A = 0x03;
    TIMSK |= (1 << OCIE0A) |        // Interrupt on compare match A
             (0 << OCIE0B) | (0 << TOIE0);
    sei(); 
}

int main(void) {
    uint8_t i = 0;
    DDRB |= (1 << PB4);
    DDRB |= (1 << PB3);
    DDRB |= (1 << PB0) | (1 << PB1);
    PORTB |= (1 << PB4);

    init_timer();
    while(1) {
        PORTB &= ~(0x3);
        PORTB |= (TCNT0 & 0x3);
        _delay_ms(10);
        i++;
        if(i == 100) {
            i = 0;
            PORTB ^= (1 << PB3);
        }
    }

    return 0;
} 
