#include <avr/io.h>
#include <util/delay.h>

#define NUM_LED     6
#define REPEAT_PATTERN  50

static uint8_t led_map[][3] = {
    { PB0, PB1, PB3 }, { PB1, PB0, PB3 },
    { PB1, PB3, PB0 }, { PB3, PB1, PB0 },
    { PB0, PB3, PB1 }, { PB3, PB0, PB1 }
};

void glow_led(uint8_t idx) {
    DDRB |= ( ( 1 << led_map[idx][0]) | (1 << led_map[idx][1]) );
    DDRB &= ~( 1 << led_map[idx][2]);
    PORTB |= ( 1 << led_map[idx][0]);
    PORTB &= ~( (1 << led_map[idx][1]) | (1 << led_map[idx][2]) );
}

void glow_pattern(unsigned int data) {
    unsigned int mask;
    unsigned char i, r;

    for(r = 0; r < REPEAT_PATTERN; r++) {
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
