#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define I2C_SCL                 (1 << PB2)
#define I2C_SDA                 (1 << PB0)
#define LED_PIN                 (1 << PB1)

#define TIM0_COMP_A_FLAG        (1 << OCF0A)
#define TIM0_CTC_MODE           (1 << WGM01)
#define TIM0_PRESCALER          (1 << CS00)
#define TIM0_COMP_A_INTR_EN     (1 << OCIE0A)

#define USI_TWO_WIRE_MODE       (1 << USIWM1)
#define USI_CLK_TIM0_COMP       (1 << USICS0)
#define USI_CNT_OVF_FLAG        (1 << USIOIF)
#define USI_CNT_OVF_INT_EN      (1 << USIOIE)
#define USI_TOGGLE_CLOCK        (1 << USITC)
#define USI_START_COND_INT_EN   (1 << USISIE)
#define USI_START_COND_FLAG     (1 << USISIF)

volatile uint8_t start_condition_detected = 0;

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

    // 8 bytes have been sent. disable the timer
    // until next byte is sent
    TCCR0B &= ~TIM0_PRESCALER;

    // Clock for ACK/NACK
    clock_pulse();

    // Stop condition
    _delay_us(10);
}

ISR(USI_START_vect) {
    USISR |= USI_START_COND_FLAG;
    start_condition_detected = 1;
}

void send_byte(uint8_t byte) {
    USISR |= USI_CNT_OVF_FLAG | 8;

    // Keep SDA low for a particular time
    _delay_us(4);

    // At the moment we set the data in USIDR, the SDA
    // become MSB of the data. this will happen even before
    // the compare match. So we will need to give a clock
    // pulse in SCL pin manually after timer started.
    USIDR = byte;

    // Start timer
    TCCR0B |= TIM0_PRESCALER;

    // The MSB will be at SDA even before the first
    // TC0 compare match(before the clock). Generate
    // a clock pulse manually
    _delay_us(3);
    USICR |= USI_TOGGLE_CLOCK;
    _delay_us(4);
    USICR |= USI_TOGGLE_CLOCK;
}

int main() {
    // Set port as output and set it as high
    DDRB  |= I2C_SDA | I2C_SCL | LED_PIN;
    PORTB |= I2C_SDA | I2C_SCL | LED_PIN;

    // Timer/Counter0 Compare mode; prescaler 1024,
    // Compare match at 255 (Maximum delay)
    TCCR0A |= TIM0_CTC_MODE;
    TIMSK  |= TIM0_COMP_A_INTR_EN;
    OCR0A  = 0x9;

    // Enable global interrupt
    sei();

    _delay_us(5);

    // Enabling USI in two wire mode, , this will set the SCL based on the
    // PORTB register value (here TRUE) and SDA based on
    // the MSB of the USIDR (here FALSE because no data
    // set so far).
    USISR |= USI_START_COND_FLAG;
    USICR |= USI_TWO_WIRE_MODE | 
                USI_CNT_OVF_INT_EN |
                USI_CLK_TIM0_COMP  |
                USI_START_COND_INT_EN;

    while(!start_condition_detected) {}
    send_byte(0xAA);

    while(1) {
        _delay_ms(500);
    }
}
