#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define I2C_SCL                 (1 << PB2)
#define I2C_SDA                 (1 << PB0)
#define LED_PIN                 (1 << PB1)

#define TIM0_CTC_MODE           (1 << WGM01)
#define TIM0_PRESCALER          ((1 << CS00));
#define TIM0_COMP_A_INTR_EN     (1 << OCIE0A)

#define USI_TWO_WIRE_MODE       (1 << USIWM1)
#define USI_CLK_TIM0_COMP       (1 << USICS0)
#define USI_CNT_OVF_FLAG        (1 << USIOIF)
#define USI_CNT_OVF_INT_EN      (1 << USIOIE)
#define USI_TOGGLE_CLOCK        (1 << USITC)

static volatile uint8_t bus_free = 0;

static inline void clock_pulse() {
    _delay_us(3);
    USICR |= USI_TOGGLE_CLOCK;
    _delay_us(4);
    USICR |= USI_TOGGLE_CLOCK;
}

ISR(TIM0_COMPA_vect) {
    clock_pulse();
}

ISR(USI_OVF_vect) {
    USISR |= USI_CNT_OVF_FLAG;
    TCCR0B &= ~TIM0_PRESCALER;
    _delay_us(3);
    clock_pulse();

    _delay_us(3);
    USICR |= USI_TOGGLE_CLOCK;
    _delay_us(4);
    bus_free = 1;
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
    OCR0A  = 0x9;

    // Enable global interrupt
    sei();
    USIDR = 0xA0;
    PORTB  |= I2C_SDA;
    TCCR0B |= TIM0_PRESCALER;
    clock_pulse();

    while(!bus_free);
    bus_free = 0;
    USICR |= USI_TOGGLE_CLOCK;
    _delay_us(3);
    USIDR = 0x00;
    TCCR0B |= TIM0_PRESCALER;
    clock_pulse();

    while(!bus_free);
    bus_free = 0;
    USICR |= USI_TOGGLE_CLOCK;
    _delay_us(3);
    USIDR = 0xAB;
    TCCR0B |= TIM0_PRESCALER;
    clock_pulse();

    while(1);
}
