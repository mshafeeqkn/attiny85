#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define I2C_SCL                 (1 << PB2)
#define I2C_SDA                 (1 << PB0)

#define TIM0_COMP_A_FLAG        (1 << OCF0A)
#define TIM0_CTC_MODE           (1 << WGM01)
#define TIM0_PRESCALER          (1 << CS01)
#define TIM0_COMP_A_INTR_EN     (1 << OCIE0A)

volatile uint8_t data = 0;
volatile uint8_t count = 8;
volatile uint8_t done = 0;

ISR(TIM0_COMPA_vect) {
    TIFR |= TIM0_COMP_A_FLAG;
    if(count == 0) {
        TCCR0B &= ~TIM0_PRESCALER;
        PORTB |= I2C_SDA;
        done = 1;
        count = 8;
    } else {
        count--;
        PORTB |= ((data & 1) << PB0);
        data >>= 1;
    }

    _delay_us(2);
    PORTB |= I2C_SCL;
    _delay_us(4);
    PORTB &= ~I2C_SCL;
    _delay_us(2);

    if(count != 0) {
        PORTB &= ~1;
    }
}

uint8_t reverse_byte(uint8_t byte) {
    byte = ((byte & 0x55) << 1) | ((byte & 0xAA) >> 1);
    byte = ((byte & 0x33) << 2) | ((byte & 0xCC) >> 2);
    byte = ((byte & 0x0F) << 4) | ((byte & 0xF0) >> 4);
    return byte;
}

void send_byte(uint8_t byte) {
    data = reverse_byte(byte);
    TCCR0B |= TIM0_PRESCALER;
    while(done != 1);
    done = 0;
}

void write_eeprom(uint8_t dev, uint8_t addr, uint8_t *data, uint8_t len) {
    int i = 0;
    uint8_t end_addr;

    while(i < len) {
        end_addr= addr | 0x0F;
        PORTB &= ~(I2C_SDA);
        _delay_us(5);
        PORTB &= ~(I2C_SCL);

        send_byte(dev << 1);
        send_byte(addr);
        for(; (addr <= end_addr && i < len); i++, addr++) {
            send_byte(data[i]);
        }

        PORTB |= I2C_SCL;
        _delay_us(5);
        PORTB |= I2C_SDA;
        _delay_ms(6);
    }
}

void init_i2c() {
    DDRB  |= I2C_SDA | I2C_SCL;
    PORTB |= I2C_SDA | I2C_SCL;

    TCCR0A |= TIM0_CTC_MODE;
    TIMSK  |= TIM0_COMP_A_INTR_EN;
    OCR0A  = 0x9;
    sei();
}

int main() {

    init_i2c();
    write_eeprom(0x50, 0x10,
        (uint8_t*)"This is a sample string to test a very long"
                  "length string having around 90 charecter length", 90);

    while(1);
}
