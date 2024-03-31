/*
 * File:   uart_rx.c
 * Author: Mohammed Shafeeque
 *
 * Description:
 * This file contains the implementation UART using USI for receivinging data
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

#define     DATA_IN_PIN         PB0
#define     FALSE               0
#define     TRUE                1

#define BAUD_RATE               2400
#if F_CPU/BAUD_RATE > 255
#define     DIVISOR             8
#else
#define     DIVISOR             1
#endif

#define     FULL_BIT_WIDTH      (F_CPU / (BAUD_RATE * DIVISOR))
#define     HALF_BIT_WIDTH      (FULL_BIT_WIDTH / 2)
#if 0
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
#endif

void test_fun() {
    PORTB |= (1 << PB2);
    _delay_ms(100);
    PORTB &= ~(1 << PB2);
    _delay_ms(100);
    PORTB |= (1 << PB2);
    _delay_ms(100);
    PORTB &= ~(1 << PB2);
    _delay_ms(100);
}

static void setup_usi_uart() {
    DDRB &= ~(1 << DATA_IN_PIN);               // Set DI pin as input
    PORTB |= (1 << DATA_IN_PIN);               // Set the output high in idle
    USICR = 0;                                 // Start with USI disabled
    GIMSK |= (1 << PCIE);                      // Enable pin change interrupt
    PCMSK |= (1 << PCINT0);                    // Enable interrupt for PB0
    sei();                                     // Enable global interrupt
}

void on_uart_data_rx() {
    TCCR0A |= (1 << WGM01);                 // Clear timer on compare match
    TCCR0B |= (1 << CS01);                  // Set prescaler 8 to clk
    GTCCR  |= (1 << PSR0);                  // Reset prescaler
    OCR0A = HALF_BIT_WIDTH;            // TODO: find the correct value to go to the middle of bit 0
    TCNT0 = 0;                              // Start counting from 0
    TIFR |= (1 << OCF0A);                   // Clear Timer0 compare match interrupt flag
    TIMSK |= (1 << OCIE0A);                 // Enable Timer0 compare match interrupt
}

ISR(PCINT0_vect) {
    GIMSK &= ~(1 << PCIE);                  // Disable the pin changeinterrupt
    on_uart_data_rx();
}

ISR(TIMER0_COMPA_vect) {
    TIMSK &= ~(1 << OCIE0A);                // Enable Timer0 compare match interrupt

    TCNT0 = 0;
    OCR0A = FULL_BIT_WIDTH;
    USICR |= (1 << USIOIE) |                    // Enabled overflow interrupt
             (0 << USIWM1) | (1 << USIWM0) |    // Enable 3 wire mode - data input to PB0
             (0 << USICS1) | (1 << USICS0) | (0 << USICLK);  // Select Timer0 Compare match

    USISR |= (1 << USIOIF) | 8;                 // Clear overflow interrupt flag
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

ISR (USI_OVF_vect) {
    uint8_t data = USIDR;
    USICR = 0;
    _delay_ms(1);
    PORTB &= ~(1 << PB2);
    if(data == 0x01) {
    } else {
        PORTB |= ((reverse_byte(data) & 1) << PB2);
    }
    GIFR |= (1 << PCIF);                    // Clear pin change interrupt flag
    GIMSK |= (1 << PCIE);                   // Enable pin change interrupt
}

#if 0
/**
 * @brief Initialize sending a single byte
 *
 * The USI over flow ISR will be invoked once the first part of total data sent.
 *
 * @param  data Data to be sent
 * @return
 */
static uint8_t uart_recv_byte() {
    while(PORTB & 0x01 == 1);                   // Wait until the line become low(start bit)

    /* Configure timer/counter0 to use as the clock of USI */
    TCCR0A |= (1 << WGM01);                     // Clear timer on compare match
    TCCR0B |= (1 << CS01);                      // Set prescaler to clk
    GTCCR  |= (1 << PSR0);                      // Reset prescaler
    OCR0A = F_CPU/(BAUD_RATE * DIVISOR);        // 1000000/(2400 * 8) = 52
    TCNT0 = 0;                                  // Count Timer/Counter0 from 0

    /* Configure USI 3 wire mode and use PB2 as data output pin */
    USICR |= (1 << USIOIE) |                    // Enabled overflow interrupt
             (0 << USIWM1) | (1 << USIWM0) |    // Enable 3 wire mode - data output to PB2
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

#endif
/**
 * The main function
 */
int main(void) {
    // uint8_t data;
    DDRB |= (1 << PB2);
    PORTB &= ~(1 << PB2);
    _delay_ms(1000);
    PORTB |= (1 << PB2);
    setup_usi_uart();

    while(1) {
#if 0
        data = uart_recv_byte();
        if(data == 0x10) {
            PORTB != (1 << PB2);
        } else {
            PORTB &= ~(1 << PB2);
        }
#endif
    }
    return 0;
}
