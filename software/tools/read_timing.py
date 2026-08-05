#!/usr/bin/env python3
import argparse
import glob
import re
import statistics
import time

import serial


TIMING_RE = re.compile(rb"(\w+)_n=(\d+) avg=([0-9.]+) min=(\d+) max=(\d+)")
RATE_RE = re.compile(
    rb"loop_hz=([0-9.]+) ctrl2_us=([0-9.]+) loops=(\d+)"
    rb"(?: flags=0x([0-9A-Fa-f]+))? dropped=(\d+)"
)

NAMES = ("adc", "enc", "foc", "posvel", "ctrl2")
MARKERS = (b"TIMING us", b"RATE ", b"STARTUP ")


def default_port():
    ports = sorted(
        glob.glob("/dev/serial/by-id/*")
        + glob.glob("/dev/ttyACM*")
        + glob.glob("/dev/ttyUSB*")
    )
    if not ports:
        raise SystemExit("No serial port found. Pass --port /dev/ttyACM0")
    return ports[0]


def parse_timing_line(line):
    if b"TIMING us" not in line:
        return None

    parsed = {}
    for match in TIMING_RE.finditer(line):
        name = match.group(1).decode("ascii")
        parsed[name] = {
            "n": int(match.group(2)),
            "avg": float(match.group(3)),
            "min": int(match.group(4)),
            "max": int(match.group(5)),
        }
    return parsed or None


def parse_rate_line(line):
    if b"RATE " not in line:
        return None

    match = RATE_RE.search(line)
    if not match:
        return None

    return {
        "loop_hz": float(match.group(1)),
        "ctrl2_us": float(match.group(2)),
        "loops": int(match.group(3)),
        "flags": int(match.group(4) or b"0", 16),
        "dropped": int(match.group(5)),
    }


def find_next_marker(buffer):
    positions = [buffer.find(marker) for marker in MARKERS]
    positions = [pos for pos in positions if pos >= 0]
    return min(positions) if positions else -1


def capture_measurements(port, baud, seconds):
    deadline = time.monotonic() + seconds
    buffer = bytearray()
    timing_samples = []
    rate_samples = []

    with serial.Serial(port, baud, timeout=0.02) as ser:
        ser.reset_input_buffer()
        while time.monotonic() < deadline:
            chunk = ser.read(4096)
            if not chunk:
                continue
            buffer.extend(chunk)

            while True:
                start = find_next_marker(buffer)
                if start < 0:
                    del buffer[:-16]
                    break
                if start:
                    del buffer[:start]

                end = buffer.find(b"\n")
                if end < 0:
                    break

                line = bytes(buffer[:end]).rstrip(b"\r")
                del buffer[: end + 1]
                timing = parse_timing_line(line)
                rate = parse_rate_line(line)
                if timing:
                    timing_samples.append(timing)
                    print(line.decode("ascii", errors="replace"))
                elif rate:
                    rate_samples.append(rate)
                    print(line.decode("ascii", errors="replace"))
                elif line.startswith(b"STARTUP "):
                    print(line.decode("ascii", errors="replace"))

    return timing_samples, rate_samples


def print_summary(samples, discarded):
    if not samples:
        raise SystemExit("No TIMING lines captured.")

    print()
    if discarded:
        print(f"Summary [us] after discarding first {discarded} window(s)")
    else:
        print("Summary [us]")
    for name in NAMES:
        entries = [sample[name] for sample in samples if sample.get(name, {}).get("n", 0) > 0]
        avgs = [entry["avg"] for entry in entries]
        mins = [entry["min"] for entry in entries]
        maxs = [entry["max"] for entry in entries]
        counts = [entry["n"] for entry in entries]
        if not avgs:
            print(f"{name:6s} no samples")
            continue
        print(
            f"{name:6s} avg={statistics.mean(avgs):7.2f} "
            f"min={min(mins):4d} max={max(maxs):4d} "
            f"windows={len(avgs)} n_mean={statistics.mean(counts):.0f}"
        )


def print_rate_summary(samples, discarded):
    if not samples:
        return

    loop_hz = [sample["loop_hz"] for sample in samples]
    ctrl2_us = [sample["ctrl2_us"] for sample in samples]
    loops = [sample["loops"] for sample in samples]
    flags = [sample["flags"] for sample in samples]
    dropped = [sample["dropped"] for sample in samples]

    print()
    if discarded:
        print(f"Rate summary after discarding first {discarded} window(s)")
    else:
        print("Rate summary")
    print(
        f"loop_hz avg={statistics.mean(loop_hz):.1f} "
        f"min={min(loop_hz):.1f} max={max(loop_hz):.1f} windows={len(loop_hz)}"
    )
    print(
        f"ctrl2_us avg={statistics.mean(ctrl2_us):.2f} "
        f"min={min(ctrl2_us):.2f} max={max(ctrl2_us):.2f} "
        f"loops_mean={statistics.mean(loops):.0f} "
        f"flags_last=0x{flags[-1]:02X} dropped_last={dropped[-1]}"
    )


def main():
    parser = argparse.ArgumentParser(description="Extract firmware timing summaries from mixed telemetry.")
    parser.add_argument("seconds", type=float, nargs="?", default=6.0)
    parser.add_argument("--port", default=None, help="serial port, e.g. /dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--discard-first", type=int, default=0, help="ignore this many initial summary windows")
    args = parser.parse_args()

    port = args.port or default_port()
    print(f"Reading timing/rate summaries from {port} for {args.seconds:g}s")
    timing_samples, rate_samples = capture_measurements(port, args.baud, args.seconds)
    if args.discard_first > 0:
        timing_samples = timing_samples[args.discard_first :]
        rate_samples = rate_samples[args.discard_first :]

    if timing_samples:
        print_summary(timing_samples, args.discard_first)
    if rate_samples:
        print_rate_summary(rate_samples, args.discard_first)
    if not timing_samples and not rate_samples:
        raise SystemExit("No TIMING or RATE lines captured.")


if __name__ == "__main__":
    main()
