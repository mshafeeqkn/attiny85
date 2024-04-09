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
    GTCCR  |= (1 << PSR0);                  // Reset prescaler
    OCR0A = HALF_BIT_WIDTH;                 // TODO: find the correct value to go to the middle of bit 0
    TCNT0 = 0;                              // Start counting from 0
    TIFR |= (1 << OCF0A);                   // Clear Timer0 compare match interrupt flag
    TIMSK |= (1 << OCIE0A);                 // Enable Timer0 compare match interrupt
    TCCR0B |= (1 << CS01);                  // Set prescaler 8 to clk
}

ISR(PCINT0_vect) {
    if(0 == (PINB & (1 << PB0))) {
        GIMSK &= ~(1 << PCIE);                  // Disable the pin changeinterrupt
        on_uart_data_rx();
    }
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

/**
 * The main function
 */
int main(void) {
    DDRB |= (1 << PB2);
    setup_usi_uart();

    while(1) {
    }
    return 0;
}
