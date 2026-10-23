
#define F_CPU 12000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <string.h>

#include "usbdrv/usbdrv.h"

/*
 * USBasp 10-pin header wiring:
 *   pin 1 PB3 = PS/2 Data, pin 7 PB5 = PS/2 Clock
 *   pin 5 PB2 = SPI SS (jumpered to ground on pin 8)
 *   pin 9 PB4 = Clock assist through a 1N4148, stripe toward pin 9
 *   pin 2 = +5 V, pin 10 = PS/2 Ground
 * Pins 4 and 6 are signals on this USBasp revision, not ground.
 */
#define PS2_CLOCK_BIT  PB5
#define PS2_DATA_BIT   PB3
#define PS2_SELECT_BIT PB2
#define PS2_ASSIST_BIT PB4
#define PS2_BUFFER_SIZE 16
/* Timer1 runs at 1.5 MHz; 1500 ticks requires 1 ms of bus inactivity. */
#define PS2_IDLE_TICKS 1500
#define PS2_TRACE_SIZE 16

/* Standard 63-byte USB HID boot-keyboard report descriptor. */
PROGMEM const char usbHidReportDescriptor[63] = {
    0x05, 0x01,       /* Generic Desktop */
    0x09, 0x06,       /* Keyboard */
    0xA1, 0x01,       /* Application Collection */
    0x05, 0x07,       /* Keyboard/Keypad */
    0x19, 0xE0,       /* Left Control */
    0x29, 0xE7,       /* Right GUI */
    0x15, 0x00,
    0x25, 0x01,
    0x75, 0x01,
    0x95, 0x08,
    0x81, 0x02,       /* Modifier bits */
    0x95, 0x01,
    0x75, 0x08,
    0x81, 0x01,       /* Reserved byte */
    0x95, 0x05,
    0x75, 0x01,
    0x05, 0x08,       /* LEDs */
    0x19, 0x01,
    0x29, 0x05,
    0x91, 0x02,
    0x95, 0x01,
    0x75, 0x03,
    0x91, 0x01,       /* LED padding */
    0x95, 0x06,
    0x75, 0x08,
    0x15, 0x00,
    0x25, 0x65,
    0x05, 0x07,
    0x19, 0x00,
    0x29, 0x65,
    0x81, 0x00,       /* Six key slots */
    0xC0
};

static uchar report[8] = {0};
static uchar idleRate = 0;
static uchar protocol = 1;
static uchar ledState = 0;
static uchar pressedKeys[13] = {0};
static uchar reportDirty = 1;
static uchar pauseReleasePending = 0;
static uchar releasedPrefix = 0;
static uchar extendedPrefix = 0;
static uchar pauseIndex = 0;
static unsigned int idleElapsedMs = 0;

/* Raw set-2 bytes are assembled by SPI and passed to the main loop here. */
static volatile uchar ps2Buffer[PS2_BUFFER_SIZE];
static volatile uchar ps2BufferHead = 0;
static volatile uchar ps2BufferTail = 0;
static volatile uchar ps2BufferOverflow = 0;
static volatile uchar ps2BitCount = 0;
static volatile uchar ps2Data = 0;
static volatile uchar ps2Parity = 0;
static volatile uchar ps2FrameValid = 0;
static uchar ps2StreamActive = 0;
static unsigned int ps2LastByteTime = 0;
static unsigned int ps2LastClockChange = 0;
static uchar ps2ClockLevel = 0;
static unsigned int statSpiBytes = 0;
static unsigned int statValidFrames = 0;
static uchar statParityErrors = 0;
static uchar statStopErrors = 0;
static uchar statAssistFlushes = 0;
static uchar statAssistFailures = 0;
static uchar statQueueOverflows = 0;
static uchar statFrameErrors = 0;
static uchar statBufferErrors = 0;
static uchar ps2Trace[PS2_TRACE_SIZE];
static uchar ps2TraceHead = 0;
static uchar ps2TraceCount = 0;
static uchar diagnosticReply[8];

/* Unprefixed IBM set-2 scan codes indexed directly to USB HID usages. */
static PROGMEM const uchar set2ToHid[256] = {
    [0x01] = 0x42, [0x03] = 0x3e, [0x04] = 0x3c, [0x05] = 0x3a,
    [0x06] = 0x3b, [0x07] = 0x45, [0x09] = 0x43, [0x0a] = 0x41,
    [0x0b] = 0x3f, [0x0c] = 0x3d, [0x0d] = 0x2b, [0x0e] = 0x35,
    [0x15] = 0x14, [0x16] = 0x1e, [0x1a] = 0x1d, [0x1b] = 0x16,
    [0x1c] = 0x04, [0x1d] = 0x1a, [0x1e] = 0x1f, [0x21] = 0x06,
    [0x22] = 0x1b, [0x23] = 0x07, [0x24] = 0x08, [0x25] = 0x21,
    [0x26] = 0x20, [0x29] = 0x2c, [0x2a] = 0x19, [0x2b] = 0x09,
    [0x2c] = 0x17, [0x2d] = 0x15, [0x2e] = 0x22, [0x31] = 0x11,
    [0x32] = 0x05, [0x33] = 0x0b, [0x34] = 0x0a, [0x35] = 0x1c,
    [0x36] = 0x23, [0x3a] = 0x10, [0x3b] = 0x0d, [0x3c] = 0x18,
    [0x3d] = 0x24, [0x3e] = 0x25, [0x41] = 0x36, [0x42] = 0x0e,
    [0x43] = 0x0c, [0x44] = 0x12, [0x45] = 0x27, [0x46] = 0x26,
    [0x49] = 0x37, [0x4a] = 0x38, [0x4b] = 0x0f, [0x4c] = 0x33,
    [0x4d] = 0x13, [0x4e] = 0x2d, [0x52] = 0x34, [0x54] = 0x2f,
    [0x55] = 0x2e, [0x58] = 0x39, [0x5a] = 0x28, [0x5b] = 0x30,
    [0x5d] = 0x31, [0x66] = 0x2a, [0x69] = 0x59, [0x6b] = 0x5c,
    [0x6c] = 0x5f, [0x70] = 0x62, [0x71] = 0x63, [0x72] = 0x5a,
    [0x73] = 0x5d, [0x74] = 0x5e, [0x75] = 0x60, [0x76] = 0x29,
    [0x77] = 0x53, [0x78] = 0x44, [0x79] = 0x57, [0x7a] = 0x5b,
    [0x7b] = 0x56, [0x7c] = 0x55, [0x7d] = 0x61, [0x7e] = 0x47,
    [0x83] = 0x40
};

static PROGMEM const uchar pauseTail[7] = {
    0x14, 0x77, 0xe1, 0xf0, 0x14, 0xf0, 0x77
};

static void ps2QueueByte(uchar value)
{
    uchar next = (ps2BufferHead + 1) & (PS2_BUFFER_SIZE - 1);

    statValidFrames++;
    ps2Trace[ps2TraceHead] = value;
    ps2TraceHead = (ps2TraceHead + 1) & (PS2_TRACE_SIZE - 1);
    if (ps2TraceCount < PS2_TRACE_SIZE)
        ps2TraceCount++;

    if (next != ps2BufferTail) {
        ps2Buffer[ps2BufferHead] = value;
        ps2BufferHead = next;
    } else {
        ps2BufferOverflow = 1;
        statQueueOverflows++;
        statBufferErrors++;
    }
}

static void ps2ConsumeBit(uchar bit)
{
    /* A PS/2 frame is start, eight LSB-first data bits, odd parity, stop. */
    if (ps2BitCount == 0) {
        if (bit == 0) {
            ps2Data = 0;
            ps2Parity = 0;
            ps2FrameValid = 1;
            ps2BitCount = 1;
        }
        return;
    }

    if (ps2BitCount <= 8) {
        if (bit) {
            ps2Data |= 1 << (ps2BitCount - 1);
            ps2Parity ^= 1;
        }
    } else if (ps2BitCount == 9) {
        if (bit)
            ps2Parity ^= 1;
        if (ps2Parity != 1) {
            statParityErrors++;
            ps2FrameValid = 0;
        }
    } else {
        if (!bit)
            statStopErrors++;
        if (bit && ps2FrameValid)
            ps2QueueByte(ps2Data);
        else {
            ps2BufferOverflow = 1;
            statFrameErrors++;
            statBufferErrors++;
        }
        ps2BitCount = 0;
        return;
    }

    ps2BitCount++;
}

static void ps2ConsumeBits(uchar bits)
{
    for (uchar i = 0; i < 8; i++, bits >>= 1)
        ps2ConsumeBit(bits & 1);
}

static uchar ps2ReadSpi(uchar *bits)
{
    uchar status;

    if (!(SPSR & (1 << SPIF)))
        return 0;

    /* AVR clears SPIF only when SPSR is read before SPDR. */
    status = SPSR;
    *bits = SPDR;
    (void)status;
    SPDR = 0;
    statSpiBytes++;
    return 1;
}

static void ps2ServiceSpi(void)
{
    uchar bits;
    uchar clockLevel = (PINB & (1 << PS2_CLOCK_BIT)) != 0;

    if (clockLevel != ps2ClockLevel) {
        ps2ClockLevel = clockLevel;
        ps2LastClockChange = TCNT1;
    }

    while (ps2ReadSpi(&bits)) {
        ps2ConsumeBits(bits);
        ps2LastByteTime = TCNT1;
        ps2StreamActive = 1;
    }

    if (!ps2StreamActive || !ps2ClockLevel ||
        (unsigned int)(TCNT1 - ps2LastByteTime) < PS2_IDLE_TICKS ||
        (unsigned int)(TCNT1 - ps2LastClockChange) < PS2_IDLE_TICKS)
        return;

    /*
     * SPI reports only complete groups of eight clocks, while PS/2 frames
     * contain eleven bits. After the keyboard goes idle, PB4/MISO pulls the
     * Clock line low through the diode to finish the pending SPI byte. SS must
     * remain grounded or the AVR tri-states MISO and these pulses disappear.
     */
    statAssistFlushes++;
    for (uchar pulses = 0;
         !(SPSR & (1 << SPIF)) && pulses < 8;
         pulses++) {
        DDRB |= 1 << PS2_ASSIST_BIT;
        _delay_us(10);
        DDRB &= ~(1 << PS2_ASSIST_BIT);
        _delay_us(10);
    }

    if (ps2ReadSpi(&bits)) {
        ps2ConsumeBits(bits);
    } else {
        SPCR &= ~(1 << SPE);
        SPCR |= 1 << SPE;
        ps2BitCount = 0;
        ps2BufferOverflow = 1;
        statAssistFailures++;
        statBufferErrors++;
    }

    ps2StreamActive = 0;
    ps2ClockLevel = 1;
    ps2LastClockChange = TCNT1;
}

static uchar ps2Read(uchar *value)
{
    if (ps2BufferHead == ps2BufferTail)
        return 0;

    *value = ps2Buffer[ps2BufferTail];
    ps2BufferTail = (ps2BufferTail + 1) & (PS2_BUFFER_SIZE - 1);
    return 1;
}

static void rebuildReport(void)
{
    uchar count = 0;

    memset(report + 2, 0, 6);
    for (uchar usage = 4; usage <= 0x65; usage++) {
        if (pressedKeys[usage >> 3] & (1 << (usage & 7))) {
            if (count < 6)
                report[2 + count] = usage;
            count++;
        }
    }

    if (count > 6)
        memset(report + 2, 1, 6);
    reportDirty = 1;
}

static void releaseAllKeys(void)
{
    memset(pressedKeys, 0, sizeof(pressedKeys));
    memset(report, 0, sizeof(report));
    releasedPrefix = 0;
    extendedPrefix = 0;
    pauseIndex = 0;
    pauseReleasePending = 0;
    reportDirty = 1;
}

static uchar extendedUsage(uchar scanCode)
{
    switch (scanCode) {
        case 0x1f: return 0xe3; /* Left GUI */
        case 0x27: return 0xe7; /* Right GUI */
        case 0x2f: return 0x65; /* Application */
        case 0x4a: return 0x54; /* Keypad / */
        case 0x5a: return 0x58; /* Keypad Enter */
        case 0x69: return 0x4d; /* End */
        case 0x6b: return 0x50; /* Left */
        case 0x6c: return 0x4a; /* Home */
        case 0x70: return 0x49; /* Insert */
        case 0x71: return 0x4c; /* Delete */
        case 0x72: return 0x51; /* Down */
        case 0x74: return 0x4f; /* Right */
        case 0x75: return 0x52; /* Up */
        case 0x7a: return 0x4e; /* Page Down */
        case 0x7c: return 0x46; /* Print Screen */
        case 0x7d: return 0x4b; /* Page Up */
        default: return 0;
    }
}

static void updateKey(uchar usage, uchar released)
{
    if (usage >= 0xe0 && usage <= 0xe7) {
        uchar mask = 1 << (usage - 0xe0);
        if (released)
            report[0] &= ~mask;
        else
            report[0] |= mask;
        reportDirty = 1;
    } else if (usage >= 4 && usage <= 0x65) {
        uchar mask = 1 << (usage & 7);
        if (released)
            pressedKeys[usage >> 3] &= ~mask;
        else
            pressedKeys[usage >> 3] |= mask;
        rebuildReport();
    }
}

static void processScanCode(uchar scanCode)
{
    uchar usage;

    if (pauseIndex) {
        if (scanCode != pgm_read_byte(&pauseTail[pauseIndex - 1])) {
            pauseIndex = 0;
            releasedPrefix = 0;
            extendedPrefix = 0;
            return;
        }
        if (++pauseIndex == 8) {
            pauseIndex = 0;
            updateKey(0x48, 0);
            pauseReleasePending = 1;
        }
        return;
    }
    if (scanCode == 0xe1) {
        pauseIndex = 1;
        releasedPrefix = 0;
        extendedPrefix = 0;
        return;
    }
    if (scanCode == 0xe0) {
        extendedPrefix = 1;
        return;
    }
    if (scanCode == 0xf0) {
        releasedPrefix = 1;
        return;
    }
    if (scanCode == 0xaa) {
        releaseAllKeys();
        return;
    }
    if (scanCode == 0xfa) {
        releasedPrefix = 0;
        extendedPrefix = 0;
        return;
    }

    if (scanCode == 0x12 || scanCode == 0x59)
        usage = extendedPrefix ? 0 : (scanCode == 0x12 ? 0xe1 : 0xe5);
    else if (scanCode == 0x14)
        usage = extendedPrefix ? 0xe4 : 0xe0;
    else if (scanCode == 0x11)
        usage = extendedPrefix ? 0xe6 : 0xe2;
    else
        usage = extendedPrefix ? extendedUsage(scanCode)
                               : pgm_read_byte(&set2ToHid[scanCode]);

    if (usage)
        updateKey(usage, releasedPrefix);
    releasedPrefix = 0;
    extendedPrefix = 0;
}

static usbMsgLen_t diagnosticRequest(uchar request, uchar page)
{
    /* Vendor requests expose counters and the last 16 accepted scan bytes. */
    memset(diagnosticReply, 0, sizeof(diagnosticReply));

    if (request == 0x40) {
        diagnosticReply[0] = 1;
        diagnosticReply[1] = statSpiBytes;
        diagnosticReply[2] = statSpiBytes >> 8;
        diagnosticReply[3] = statValidFrames;
        diagnosticReply[4] = statValidFrames >> 8;
        diagnosticReply[5] = statParityErrors;
        diagnosticReply[6] = statStopErrors;
        diagnosticReply[7] = statAssistFlushes;
    } else if (request == 0x41) {
        diagnosticReply[0] = statBufferErrors;
        diagnosticReply[1] = ps2TraceCount;
        diagnosticReply[2] = ps2BitCount;
        diagnosticReply[3] = ps2StreamActive;
        diagnosticReply[4] = (ps2BufferHead - ps2BufferTail) &
                             (PS2_BUFFER_SIZE - 1);
        diagnosticReply[5] = statAssistFailures;
        diagnosticReply[6] = statQueueOverflows;
        diagnosticReply[7] = statFrameErrors;
    } else if (request == 0x42) {
        uchar first = (ps2TraceHead - ps2TraceCount) &
                      (PS2_TRACE_SIZE - 1);
        uchar offset = page ? 8 : 0;

        for (uchar i = 0; i < 8 && offset + i < ps2TraceCount; i++)
            diagnosticReply[i] = ps2Trace[(first + offset + i) &
                                          (PS2_TRACE_SIZE - 1)];
    } else if (request == 0x43) {
        statSpiBytes = 0;
        statValidFrames = 0;
        statParityErrors = 0;
        statStopErrors = 0;
        statAssistFlushes = 0;
        statAssistFailures = 0;
        statQueueOverflows = 0;
        statFrameErrors = 0;
        statBufferErrors = 0;
        ps2TraceHead = 0;
        ps2TraceCount = 0;
    }

    usbMsgPtr = diagnosticReply;
    return sizeof(diagnosticReply);
}

usbMsgLen_t usbFunctionSetup(uchar data[8])
{
    usbRequest_t *rq = (void *)data;

    if ((rq->bmRequestType & USBRQ_TYPE_MASK) == USBRQ_TYPE_VENDOR &&
        rq->bRequest >= 0x40 && rq->bRequest <= 0x43)
        return diagnosticRequest(rq->bRequest, rq->wIndex.bytes[0]);

    if ((rq->bmRequestType & USBRQ_TYPE_MASK) ==
        USBRQ_TYPE_CLASS) {

        switch (rq->bRequest) {
            case 0x01: /* GET_REPORT */
                usbMsgPtr = report;
                return sizeof(report);

            case 0x02: /* GET_IDLE */
                usbMsgPtr = &idleRate;
                return 1;

            case 0x03: /* GET_PROTOCOL */
                usbMsgPtr = &protocol;
                return 1;

            case 0x09: /* SET_REPORT: accept LED state */
                return USB_NO_MSG;

            case 0x0A: /* SET_IDLE */
                idleRate = rq->wValue.bytes[1];
                return 0;

            case 0x0B: /* SET_PROTOCOL */
                protocol = rq->wValue.bytes[0];
                return 0;
        }
    }

    return 0;
}

uchar usbFunctionWrite(uchar *data, uchar len)
{
    if (len > 0)
        ledState = data[0];

    return 1;
}

int main(void)
{
    /* Hold PS/2 Clock low while the keyboard selects AT/PS/2 mode. */
    PORTB = 0;
    DDRB = 1 << PS2_CLOCK_BIT;

    cli();

    /* USBasp also connects D+ to PD2 for the INT0 input. */
    DDRD &= ~(1 << PD2);
    PORTD &= ~(1 << PD2);
    usbInit();
    usbDeviceDisconnect();

    for (unsigned int i = 0; i < 2000; i++)
        _delay_ms(1);

    usbDeviceConnect();

    /* Header pin 5 must be jumpered to confirmed ground on pin 8. */
    TCCR1B = 1 << CS11;
    DDRB &= ~((1 << PS2_SELECT_BIT) | (1 << PS2_DATA_BIT) |
              (1 << PS2_ASSIST_BIT));
    SPDR = 0;
    SPCR = (1 << SPE) | (1 << DORD) | (1 << CPOL);
    DDRB &= ~(1 << PS2_CLOCK_BIT);
    ps2ClockLevel = (PINB & (1 << PS2_CLOCK_BIT)) != 0;
    ps2LastClockChange = TCNT1;

    sei();

    /* Timer0: approximately 21.8 ms per overflow. */
    TCCR0 = (1 << CS02) | (1 << CS00);
    TIFR = (1 << TOV0);

    for (;;) {
        uchar scanCode;

        ps2ServiceSpi();
        usbPoll();
        ps2ServiceSpi();

        if (ps2BufferOverflow) {
            cli();
            ps2BufferTail = ps2BufferHead;
            ps2BufferOverflow = 0;
            sei();
            releaseAllKeys();
        }
        while (ps2Read(&scanCode))
            processScanCode(scanCode);

        if (TIFR & (1 << TOV0)) {
            TIFR = (1 << TOV0);
            if (idleElapsedMs < 1000)
                idleElapsedMs += 22;
        }

        if (usbInterruptIsReady() &&
            (reportDirty || (idleRate && idleElapsedMs >= idleRate * 4U))) {
            usbSetInterrupt(report, sizeof(report));
            reportDirty = 0;
            idleElapsedMs = 0;

            if (pauseReleasePending) {
                pauseReleasePending = 0;
                updateKey(0x48, 1);
            }
        }
    }
}
