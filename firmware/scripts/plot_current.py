#!/usr/bin/env python3
import argparse
import glob
import re
import time

import matplotlib.pyplot as plt
import serial


LINE_RE = re.compile(
    r"M0\[A,Bcalc,C\]=([-+0-9.]+),([-+0-9.]+),([-+0-9.]+)\s+"
    r"M1\[A,Bcalc,C\]=([-+0-9.]+),([-+0-9.]+),([-+0-9.]+)"
)


def default_port():
    ports = sorted(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*"))
    if not ports:
        raise SystemExit("No serial port found. Pass --port /dev/ttyACM0")
    return ports[0]


def main():
    parser = argparse.ArgumentParser(description="Plot Pico current telemetry.")
    parser.add_argument("seconds", type=float, help="capture duration")
    parser.add_argument("--port", default=None, help="serial port, e.g. /dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--save", help="optional output image path")
    args = parser.parse_args()

    port = args.port or default_port()
    print(f"Reading {port} at {args.baud} baud for {args.seconds:g}s")

    t0 = time.monotonic()
    times = []
    currents = [[], [], [], [], [], []]

    with serial.Serial(port, args.baud, timeout=0.2) as ser:
        ser.reset_input_buffer()
        while (time.monotonic() - t0) < args.seconds:
            line = ser.readline().decode(errors="replace").strip()
            match = LINE_RE.search(line)
            if not match:
                continue

            times.append(time.monotonic() - t0)
            for channel, value in enumerate(match.groups()):
                currents[channel].append(float(value))

    if not times:
        raise SystemExit("No current telemetry lines captured.")

    labels = ["M0 A", "M0 Bcalc", "M0 C", "M1 A", "M1 Bcalc", "M1 C"]
    for values, label in zip(currents, labels):
        plt.plot(times, values, label=label)

    plt.xlabel("Time [s]")
    plt.ylabel("Current [ADC counts]")
    plt.title(f"Current telemetry from {port}")
    plt.grid(True, alpha=0.3)
    plt.legend()
    plt.tight_layout()

    if args.save:
        plt.savefig(args.save, dpi=150)
    else:
        plt.show()


if __name__ == "__main__":
    main()
