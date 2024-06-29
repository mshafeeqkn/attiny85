#include <avr/io.h>
#include <avr/interrupt.h>


#define WITH_INTERRUPT
#define ONE_SEC_OVF_COUNT   3922    // 255uS per counter overflow; 1s/255us = 3921.56


unsigned int count;
volatile unsigned char toggle_led = 0;

void init_timer0() {
    DDRB = 0x02;
    PORTB = 0x00;
 
    TCCR0B |= (1 << CS00);          // No pre-scalar
    TCNT0 = 0x00;                   // Initial counter value
#ifdef WITH_INTERRUPT
    TIMSK |= (1 << TOIE0);
    sei();
#endif
}

ISR(TIMER0_OVF_vect) {                      // Timer0 Overflow ISR
    count++;
    count = count % ONE_SEC_OVF_COUNT;      // Wait until ONE_SEC_OVF_COUNT times
    if(count == 0) {                        // interrupt happened.
        toggle_led = 1;
    }
}

int main(void) {

    init_timer0();

    while(1) {
#ifdef WITH_INTERRUPT
        // PORTB = toggle_led;
        if(toggle_led) {
            toggle_led = 0;
            PORTB ^= 0x02;
        }
#else
        count = ONE_SEC_OVF_COUNT;
        while(count != 0) {
            while((TIFR & (1 << TOV0)) == 0);
            TIFR |= (1 << TOV0);
            count--;
        }
        PORTB ^= 0x02;
#endif
    }

    return 0;
} 
