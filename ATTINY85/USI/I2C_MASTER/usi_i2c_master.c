#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define I2C_SCL                 (1 << PB2)
#define I2C_SDA                 (1 << PB0)

#define TIM0_COMP_A_FLAG        (1 << OCF0A)
#define TIM0_CTC_MODE           (1 << WGM01)
#define TIM0_PRESCALER          (1 << CS01)
#define TIM0_COMP_A_INTR_EN     (1 << OCIE0A)

#define USI_TWO_WIRE_MODE       (1 << USIWM1)
#define USI_CLK_TIM0_COMP       (1 << USICS0)
#define USI_CNT_OVF_FLAG        (1 << USIOIF)
#define USI_CNT_OVF_INT_EN      (1 << USIOIE)
#define USI_TOGGLE_CLOCK        (1 << USITC)
#define USI_START_COND_FLAG     (1 << USISIF)

volatile uint8_t complete = 0;

ISR(TIM0_COMPA_vect) {
    TIFR |= TIM0_COMP_A_FLAG;
    _delay_us(2);
    USICR |= USI_TOGGLE_CLOCK;
    _delay_us(4);
    USICR |= USI_TOGGLE_CLOCK;
}

ISR(USI_OVF_vect) {
    USIDR = 0x80;
    TIFR |= TIM0_COMP_A_FLAG;
    TCCR0B &= ~TIM0_PRESCALER;
    USISR |= USI_CNT_OVF_FLAG;
    PORTB |= I2C_SDA;
    USISR &= 0x0F;

    complete = 1;
}

int main() {
    DDRB  |= I2C_SDA | I2C_SCL;
    PORTB |= I2C_SDA | I2C_SCL;

    USIDR = 0xA1;
    USISR |= (USI_CNT_OVF_FLAG | 8);
    USICR |= (USI_TWO_WIRE_MODE |
              USI_CNT_OVF_INT_EN |
              USI_CLK_TIM0_COMP);

    TCCR0A |= TIM0_CTC_MODE;
    TIMSK  |= TIM0_COMP_A_INTR_EN;
    OCR0A  = 0x9;
    sei();

    PORTB &= ~(I2C_SDA);
    _delay_us(5);
    PORTB &= ~(I2C_SCL);
    PORTB |= I2C_SDA;
    USISR |= USI_START_COND_FLAG;

    TCCR0B |= TIM0_PRESCALER;
    _delay_us(2);
    USICR |= USI_TOGGLE_CLOCK;
    _delay_us(4);
    USICR |= USI_TOGGLE_CLOCK;

    while(complete == 0);
    complete = 0;
    _delay_us(10);
    USIDR = 0x00;
    TCCR0B |= TIM0_PRESCALER;
    USISR |= (0xF0 | 8);
    _delay_us(2);
    USICR |= USI_TOGGLE_CLOCK;
    _delay_us(4);
    USICR |= USI_TOGGLE_CLOCK;
    while(complete == 0);
    complete = 0;
    _delay_us(2);
    USICR |= USI_TOGGLE_CLOCK;
    _delay_us(4);
    USICR |= USI_TOGGLE_CLOCK;

    _delay_us(10);
    USIDR = 0xAB;
    TCCR0B |= TIM0_PRESCALER;
    USISR |= (0xF0 | 8);
    _delay_us(2);
    USICR |= USI_TOGGLE_CLOCK;
    _delay_us(4);
    USICR |= USI_TOGGLE_CLOCK;
    while(complete == 0);
    complete = 0;
    _delay_us(2);
    USICR |= USI_TOGGLE_CLOCK;
    _delay_us(4);
    USICR |= USI_TOGGLE_CLOCK;

    while(1);
}
