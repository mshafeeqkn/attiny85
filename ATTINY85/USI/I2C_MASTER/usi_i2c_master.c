#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define I2C_SCL                 (1 << PB2)
#define I2C_SDA                 (1 << PB0)
#define LED_PIN                 (1 << PB1)

#define TIM0_CTC_MODE           (1 << WGM01)
#define TIM0_PRESCALER          ((1 << CS02) | (1 << CS00));
#define TIM0_COMP_A_INTR_EN     (1 << OCIE0A)

#define USI_TWO_WIRE_MODE       (1 << USIWM1)
#define USI_CLK_TIM0_COMP       (1 << USICS0)
#define USI_CNT_OVF_FLAG        (1 << USIOIF)
#define USI_CNT_OVF_INT_EN      (1 << USIOIE)
#define USI_TOGGLE_CLOCK        (1 << USITC)

static inline void clock_pulse() {
    _delay_ms(65);
    USICR |= USI_TOGGLE_CLOCK;
    _delay_ms(130);
    USICR |= USI_TOGGLE_CLOCK;
}

ISR(TIM0_COMPA_vect) {
    clock_pulse();
}

ISR(USI_OVF_vect) {
    USISR |= USI_CNT_OVF_FLAG;
    TCCR0B &= ~TIM0_PRESCALER;
}

int main() {
    // Set port as output and set it as high
    DDRB  |= I2C_SDA | I2C_SCL | LED_PIN;

    USISR |= USI_CNT_OVF_FLAG | 8;
    USICR |= USI_TWO_WIRE_MODE |
                USI_CNT_OVF_INT_EN |
                USI_CLK_TIM0_COMP;

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
