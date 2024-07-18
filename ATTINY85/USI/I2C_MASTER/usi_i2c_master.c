#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define I2C_SCL                 (1 << PB2)
#define I2C_SDA                 (1 << PB0)
#define LED_PIN                 (1 << PB1)

#define TIM0_COMP_A_FLAG        (1 << OCF0A)
#define TIM0_CTC_MODE           (1 << WGM01)
#define TIM0_PRESCALER          (1 << CS02) | (1 << CS00);
#define TIM0_COMP_A_INTR_EN     (1 << OCIE0A)

#define USI_TWO_WIRE_MODE       (1 << USIWM1)
#define USI_CLK_TIM0_COMP       (1 << USICS0)
#define USI_CNT_OVF_FLAG        (1 << USIOIF)
#define USI_CNT_OVF_INT_EN      (1 << USIOIE)

ISR(TIM0_COMPA_vect) {
    PORTB |= I2C_SCL;
    _delay_ms(50);
    PORTB &= ~I2C_SCL;
}

ISR(USI_OVF_vect) {
    USISR |= (1 << USIOIF) | 8;
    USIDR = 0xAA;
}

int main() {
    // Set port as output and set it as high
    DDRB  |= I2C_SDA | I2C_SCL | LED_PIN;
    PORTB |= I2C_SDA | I2C_SCL | LED_PIN;

    // Timer/Counter0 Compare mode; prescaler 1024,
    // Compare match at 255 (Maximum delay)
    TCCR0A |= TIM0_CTC_MODE;
    TCCR0B |= TIM0_PRESCALER;
    TIMSK  |= TIM0_COMP_A_INTR_EN;
    OCR0A  = 0xFF;

    // Configure USI 2-wire mode, Clock on compare mode
    USICR |= USI_TWO_WIRE_MODE | 
                USI_CLK_TIM0_COMP |
                USI_CNT_OVF_INT_EN;

    // Enable global interrupt
    sei();
    while(1) {
        _delay_ms(500);
    }
}
