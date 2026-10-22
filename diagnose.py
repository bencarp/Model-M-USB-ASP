#!/usr/bin/env python3

import argparse
import sys

try:
    import usb.core
except ImportError:
    sys.exit("PyUSB is required: sudo apt install python3-usb")


def read(device, request, index=0):
    return bytes(device.ctrl_transfer(0xC0, request, 0, index, 8, timeout=1000))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--reset", action="store_true")
    args = parser.parse_args()

    device = usb.core.find(idVendor=0x16C0, idProduct=0x05DF)
    if device is None:
        sys.exit("IBM Model M USB device not found")

    if args.reset:
        read(device, 0x43)
        print("Diagnostic counters and trace reset")
        return

    stats = read(device, 0x40)
    state = read(device, 0x41)
    trace = (read(device, 0x42, 0) + read(device, 0x42, 1))[:state[1]]
    spi_bytes = stats[1] | (stats[2] << 8)
    valid_frames = stats[3] | (stats[4] << 8)

    print(f"SPI bytes:       {spi_bytes}")
    print(f"Valid frames:    {valid_frames}")
    print(f"Parity errors:   {stats[5]}")
    print(f"Stop errors:     {stats[6]}")
    print(f"Assist flushes:  {stats[7]}")
    print(f"Buffer/errors:   {state[0]}")
    print(f"Parser bitcount: {state[2]}")
    print(f"Stream active:   {state[3]}")
    print(f"Queue depth:     {state[4]}")
    print(f"Assist failures: {state[5]}")
    print(f"Queue overflows: {state[6]}")
    print(f"Frame errors:    {state[7]}")
    print("Last scan bytes: " + " ".join(f"{byte:02X}" for byte in trace))


if __name__ == "__main__":
    try:
        main()
    except usb.core.USBError as error:
        sys.exit(f"USB error: {error}; try running with sudo")
