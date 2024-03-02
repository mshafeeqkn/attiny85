Registers

Timer/Counter 0 (8bit)

TCNT0   - Timer/Counter register        - Increments the value
OCR0A   - Output Compare Register A     - Compare with value of TCNT0 in very cycle
OCR0B   - Output Compare Register B     - Compare with value of TCNT0 in very cycle

TIFR    - Timer Interrpt Flag Register  - All interrupt flags
TIMSK   - Timer Interrupt Mask Register - Mask timer interrupt individually

OCF0A   - Output Compare Flag for OCR0A
OCF0B   - Output Compare Flag for OCR0B

TCCR0A  - Timer Clock Control Register A - Counting sequence - WGM01, WGM02
TCCR0B  - Timer Clock Control Register V - Clock select bit CS0[2:0] and Counting sequence bit WGM02
CS0[2:0]    - Select clock prescalar - Timer stopped when CS0[2:0] = 0

