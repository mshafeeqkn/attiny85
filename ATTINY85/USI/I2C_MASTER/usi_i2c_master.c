#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define I2C_SCL                 (1 << PB2)
#define I2C_SDA                 (1 << PB0)
#define LED_PIN                 (1 << PB1)

#define TIM0_CTC_MODE           (1 << WGM01)
#define TIM0_PRESCALER          ((1 << CS00) | (1 << CS01))
#define TIM0_COMP_A_INTR_EN     (1 << OCIE0A)

static inline void clock_pulse() {
    _delay_ms(65);
    PORTB ^= I2C_SCL;
    _delay_ms(130);
    PORTB ^= I2C_SCL;
}

ISR(TIM0_COMPA_vect) {
    clock_pulse();
}

int main() {
    // Set port as output and set it as high
    DDRB  |= I2C_SDA | I2C_SCL | LED_PIN;
    PORTB |= I2C_SDA;

    // Timer/Counter0 Compare mode; prescaler 1024,
    // Compare match at 255 (Maximum delay)
    TCCR0A |= TIM0_CTC_MODE;
    TIMSK  |= TIM0_COMP_A_INTR_EN;
    OCR0A  = 0xFF;

    // Enable global interrupt
    sei();

    TCCR0B |= TIM0_PRESCALER;
    while(1);
}
