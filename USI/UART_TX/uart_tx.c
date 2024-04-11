/*
 * File:   uart.c
 * Author: Mohammed Shafeeque
 *
 * Description:
 * This file contains the implementation UART using USI for sending data
 */

/**
 * Copyright (c) 2024 Mohammed Shafeeque
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define     DATA_OUT_PIN        PB1
#define     FALSE               0
#define     TRUE                1

#define BAUD_RATE               2400
#if F_CPU/BAUD_RATE > 255
#define DIVISOR                 8
#else
#define DIVISOR                 1
#endif

// Enum to keep the state of output line
typedef enum {
    AVAILABLE,
    PROC_BUFF1,
    PROC_BUFF2
} line_status_t;

volatile static line_status_t line_status = AVAILABLE;

static uint8_t  data_buff;

/**
 * @brief Initialize UART, set output pins
 * 
 * @param   None
 * @return  None
 */
static void setup_usi_uart() {
    DDRB |= (1 << DATA_OUT_PIN);                // Set data out port as output
    PORTB |= (1 << DATA_OUT_PIN);               // Set the output high in idle
    sei();                                      // Enable global interrupt
}

/**
 * @brief Reverse the given byte
 * 
 * @param byte Byte data to be reversed
 * @return     Reversed byte
 */
static uint8_t reverse_byte (uint8_t byte) {
    byte = ((byte >> 1) & 0x55) | ((byte << 1) & 0xaa);
    byte = ((byte >> 2) & 0x33) | ((byte << 2) & 0xcc);
    byte = ((byte >> 4) & 0x0f) | ((byte << 4) & 0xf0);
    return byte;
}

/**
 * @brief Initialize sending a single byte
 *
 * The USI over flow ISR will be invoked once the first part of total data sent.
 *
 * @param  data Data to be sent
 * @return
 */
static void uart_send_byte(uint8_t data) {
    line_status = PROC_BUFF1;
    data_buff = reverse_byte(data);

    /* Configure timer/counter0 to use as the clock of USI */
    TCCR0A |= (1 << WGM01);                     // Clear timer on compare match
    TCCR0B |= (1 << CS01);                      // Set prescaler to clk
    GTCCR  |= (1 << PSR0);                      // Reset prescaler
    OCR0A = F_CPU/(BAUD_RATE * DIVISOR);        // 1000000/(2400 * 8) = 52
    TCNT0 = 0;                                  // Count Timer/Counter0 from 0

    /* Configure USI 3 wire mode and use PB1 as data output pin */
    USIDR = 0x00 | (data_buff >> 1);            // Set start bit 0 and append first 7 bits
    USICR |= (1 << USIOIE) |                    // Enabled overflow interrupt
             (0 << USIWM1) | (1 << USIWM0) |    // Enable 3 wire mode - data output to PB1
             (0 << USICS1) | (1 << USICS0) | (0 << USICLK);  // Select Timer0 Compare match
                                                             // as USI Clock source.

    USISR |= (1 << USIOIF) |                    // Clear overflow interrupt flag
             (16 - 8);                          // we have 8 bits to send; the overflow interrupt
                                                // trigger when 8 bits are sent. 
}

/**
 * ISR for USI counter overflow. This will be called upon completing all data in the USIDR sent.
 */
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


/**
 * @brief Send a NULL terminated string throuh UART
 *
 * @param data - Pointer to char array of data
 */
void uart_send_str(char *data) {
    while(*data != 0) {
        while( AVAILABLE != line_status );   // wait until the data is available
        uart_send_byte(*data);
        data++;
    }
}

/**
 * The main function
 */
int main(void) {
    setup_usi_uart();

    while(1) {
        uart_send_str("Permission is hereby granted, free of charge, to any person obtaining a copy of this "
                      "software and associated documentation files, to deal in the Software without "
                      "restriction, including without limitation the rights to use, copy, modify, merge, "
                      "publish, distribute, sublicense, and/or sell copies of the Software, and to permit "
                      "persons to whom the Software is furnished to do so, subject to the following conditions:");

        uart_send_byte('\r');
        uart_send_byte('\n');
        _delay_ms(1000);

    }
    return 0;
}
