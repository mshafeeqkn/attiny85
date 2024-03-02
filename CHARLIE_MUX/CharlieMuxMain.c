#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#include <CharlieMux.h>
#define NUM_MODES   3

unsigned char mode = 0;
static unsigned int data = 0x1;

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
    }
}

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
    }
}

int main(void) {
    DDRB &= ~(1 << PB2);
    MCUCR |= (1 << ISC01);
    GIMSK |= (1 << INT0);
    sei();
    while(1) {
        glow_pattern(data);
        next();
    }
}