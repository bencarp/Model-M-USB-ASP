/*
 * Passive PS/2/SPI monitor for an Arduino Mega 2560.
 *
 * D51 / PB2: USBasp MOSI, PS/2 Data
 * D50 / PB3: USBasp MISO, Clock-assist output
 * D52 / PB1: USBasp SCK, PS/2 Clock
 * GND:        USBasp GND
 *
 * The monitored pins remain inputs with their internal pull-ups disabled.
 * A pin-change ISR records every transition with Timer1 timestamps at 4 us.
 */

#include <Arduino.h>
#include <avr/interrupt.h>

#define DATA_MASK   _BV(PB2)
#define ASSIST_MASK _BV(PB3)
#define CLOCK_MASK  _BV(PB1)
#define SIGNAL_MASK (DATA_MASK | ASSIST_MASK | CLOCK_MASK)
#define CAPTURE_SIZE 384
#define IDLE_TICKS 12500U /* 50 ms at 4 us per Timer1 tick. */

struct Sample {
    uint16_t ticks;
    uint8_t pins;
};

static volatile Sample samples[CAPTURE_SIZE];
static volatile uint16_t sampleCount;
static volatile uint16_t lastEvent;
static volatile uint8_t lastPins;
static volatile uint8_t capturing;
static volatile uint8_t captureReady;
static volatile uint8_t overflowed;

ISR(PCINT0_vect)
{
    uint8_t pins = PINB & SIGNAL_MASK;
    uint16_t now = TCNT1;

    if (pins == lastPins)
        return;

    lastPins = pins;
    lastEvent = now;
    capturing = 1;

    if (sampleCount < CAPTURE_SIZE) {
        samples[sampleCount].ticks = now;
        samples[sampleCount].pins = pins;
        sampleCount++;
    } else {
        overflowed = 1;
        captureReady = 1;
        PCICR &= ~_BV(PCIE0);
    }
}

static void armCapture(void)
{
    uint8_t savedSreg = SREG;

    cli();
    sampleCount = 0;
    capturing = 0;
    captureReady = 0;
    overflowed = 0;
    lastPins = PINB & SIGNAL_MASK;
    lastEvent = TCNT1;
    PCIFR = _BV(PCIF0);
    PCICR |= _BV(PCIE0);
    SREG = savedSreg;
}

static void printLevel(uint8_t pins, uint8_t mask)
{
    Serial.print((pins & mask) ? '1' : '0');
}

static void printCapture(void)
{
    uint16_t count;
    uint16_t first;
    uint8_t didOverflow;

    noInterrupts();
    count = sampleCount;
    didOverflow = overflowed;
    first = count ? samples[0].ticks : 0;
    interrupts();

    Serial.println();
    Serial.print(F("capture,event_count="));
    Serial.print(count);
    Serial.print(F(",overflow="));
    Serial.println(didOverflow);
    Serial.println(F("time_us,clock,data,assist"));

    for (uint16_t i = 0; i < count; i++) {
        uint16_t ticks;
        uint8_t pins;

        noInterrupts();
        ticks = samples[i].ticks;
        pins = samples[i].pins;
        interrupts();

        uint16_t elapsed = ticks - first;
        Serial.print((uint32_t)elapsed * 4);
        Serial.print(',');
        printLevel(pins, CLOCK_MASK);
        Serial.print(',');
        printLevel(pins, DATA_MASK);
        Serial.print(',');
        printLevel(pins, ASSIST_MASK);
        Serial.println();
    }

    Serial.println(F("end"));
}

void setup(void)
{
    Serial.begin(115200);

    /* Inputs, no internal pull-ups: do not drive the USBasp or keyboard. */
    DDRB &= ~SIGNAL_MASK;
    PORTB &= ~SIGNAL_MASK;

    /* Timer1 free-runs at F_CPU / 64: 4 us per tick at 16 MHz. */
    TCCR1A = 0;
    TCCR1B = _BV(CS11) | _BV(CS10);
    TCNT1 = 0;

    PCMSK0 = _BV(PCINT2) | _BV(PCINT3) | _BV(PCINT1);
    armCapture();

    Serial.println(F("PS/2 SPI monitor ready"));
    Serial.println(F("Press and release one keyboard key."));
}

void loop(void)
{
    if (capturing && (uint16_t)(TCNT1 - lastEvent) >= IDLE_TICKS) {
        noInterrupts();
        captureReady = 1;
        PCICR &= ~_BV(PCIE0);
        interrupts();
    }

    if (captureReady) {
        printCapture();
        delay(100);
        armCapture();
    }
}
