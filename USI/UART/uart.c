#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define     DATA_OUT_PIN        PB1
#define     FALSE               0
#define     TRUE                1

typedef enum {
    AVAILABLE,
    PROC_BUFF1,
    PROC_BUFF2
} line_status_t;

volatile static line_status_t line_status = AVAILABLE;

static uint8_t  data_buff;

static void setup_usi_uart() {
    // Set data out port as output
    DDRB |= (1 << DATA_OUT_PIN);

    // Set the output high in idle
    PORTB |= (1 << DATA_OUT_PIN);

    sei();
}

static uint8_t reverse_byte (uint8_t byte) {
    byte = ((byte >> 1) & 0x55) | ((byte << 1) & 0xaa);
    byte = ((byte >> 2) & 0x33) | ((byte << 2) & 0xcc);
    byte = ((byte >> 4) & 0x0f) | ((byte << 4) & 0xf0);
    return byte;
}

static void send_data(uint8_t data) {
    line_status = PROC_BUFF1;
    data_buff = reverse_byte(data);

    /* Configure timer/counter0 to use as the clock of USI */
    TCCR0A |= (1 << WGM00);                     // Clear timer on compare match
    TCCR0B |= (1 << CS00);                      // Set prescaler to clk
    GTCCR  |= (1 << PSR0);                      // Reset prescaler
    OCR0A = 208;                                 // 128000/4800 = 26.667 baud rate 4800
    TCNT0 = 0;                                  // Count up from 0 

    /* Configure USI 3 wire mode and use PB1 as data output pin */
    USIDR = 0x00 | (data_buff >> 1);            // Set start bit 0 and append first 7 bits
    USICR |= (1 << USIOIE) |                    // Enabled overflow interrupt
             (1 << USIWM1) | (1 << USIWM0) |    // Enable 3 wire mode - data output to PB1
             (0 << USICS1) | (1 << USICS0) | (0 << USICLK);  // Select Timer0 Compare match as USI Clock source.

    USISR |= (1 << USIOIF) |                    // Clear overflow interrupt flag
             (16 - 8);                          // we have 8 bits to send; the overflow interrupt
                                                // trigger when 8 bits are sent. 
}

ISR (USI_OVF_vect) {
    if( PROC_BUFF1 == line_status ) {
        USIDR = 0x7F | (data_buff << 7);        // remaining bit + stop bits
        USISR = (1 << USIOIF) |                 // Clear USI overflow interrupt flag
                (16 - 2);                       // Set USI counter to send last data bit + 1 stop bits
        line_status = PROC_BUFF2;
    } else {
        PORTB |= (1 << DATA_OUT_PIN);           // Ensure output is high
        USICR = 0;                              // Disable USI.
        USISR |= (1 << USIOIF);                 // clear interrupt flag
        line_status = AVAILABLE;
    }
}

int main(void) {
    setup_usi_uart();

    while(1) {
        while( AVAILABLE != line_status );   // wait until the data is available
        send_data('M');
    }
    return 0;
}
