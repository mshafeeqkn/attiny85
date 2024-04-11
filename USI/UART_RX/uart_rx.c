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
// 53 is the number of clock ticks elapsed to
// start the timer1 after start bit is detected.
// This value is calculated with the help of timer/counter1.
#define     START_BIT_FIX       (53 / DIVISOR)

typedef enum {
    AVAILABLE,
    PROC_BUFF1,
    PROC_BUFF2
} line_status_t;
volatile static line_status_t line_status = AVAILABLE;

static uint8_t  data_buff = 0;

/**
 * @brief Blink the LED number of times
 * 
 * @param c How many times the LED is to blink?
 * @return None
 */ 
void blink_led(uint8_t c) {
    while(c) {
        PORTB |= (1 << PB4);
        _delay_ms(150);
        PORTB &= ~(1 << PB4);
        _delay_ms(350);
        c--;
    }
    data_buff = 0;
}

/**
 * @brief Initialize UART for the reception.
 * 
 * @param   None
 * @return  None
 */
static void setup_usi_uart_rx() {
    DDRB &= ~(1 << DATA_IN_PIN);               // Set DI pin as input
    PORTB |= (1 << DATA_IN_PIN);               // Set the output high in idle
    USICR = 0;                                 // Start with USI disabled
    GIMSK |= (1 << PCIE);                      // Enable pin change interrupt
    PCMSK |= (1 << PCINT0);                    // Enable interrupt for PB0
    sei();                                     // Enable global interrupt
}

/**
 * @brief Setup the timer to reach the half bit width after the start bit is detected
 * 
 * @param None
 * @param None
 */
void on_uart_data_rx() {
    TCCR0A |= (1 << WGM01);                 // Clear timer on compare match
    GTCCR  |= (1 << PSR0);                  // Reset prescaler
    OCR0A = HALF_BIT_WIDTH - START_BIT_FIX; // Set the counter value
    TCNT0 = 0;                              // Start counting from 0
    TIFR |= (1 << OCF0A);                   // Clear Timer0 compare match interrupt flag
    TIMSK |= (1 << OCIE0A);                 // Enable Timer0 compare match interrupt
    TCCR0B |= (1 << CS01);                  // Set prescaler 8 to clk
}

/**
 * ISR for start bit of the data (External interrupt).
 * This will be called upon receiving the data.
 */
ISR(PCINT0_vect) {
    // start_timer1();
    if(0 == (PINB & (1 << PB0))) {
        GIMSK &= ~(1 << PCIE);                  // Disable the pin changeinterrupt
        on_uart_data_rx();
    }
}

/**
 * ISR Timer0, This will be called after half bit width
 * is completed after the start bit is detected.
 */
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

/**
 * ISR for USI counter overflow.
 * This will be called upon completing 1 byte rx.
 */
ISR (USI_OVF_vect) {
    data_buff = USIBR;
    USICR = 0;
    GIFR |= (1 << PCIF);                        // Clear pin change interrupt flag
    GIMSK |= (1 << PCIE);                       // Enable pin change interrupt
}


/**
 * The main function
 */
int main(void) {
    DDRB |= (1 << PB4);
    setup_usi_uart_rx();

    while(1) {
        blink_led(reverse_byte(data_buff));
    }

    return 0;
}
