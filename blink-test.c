#define F_CPU 12000000UL

#include <avr/io.h>
#include <util/delay.h>

int main(void)
{
    // PB0 is ATmega8A TQFP-32 physical pin 12.
    DDRB |= (1 << PB0);

    while (1) {
        PORTB ^= (1 << PB0);
        _delay_ms(1000);
    }
}
