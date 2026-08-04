#!/usr/bin/env python3
import argparse
import glob
import struct
import time

import matplotlib.pyplot as plt
import serial


MAGIC = b"\xA5\x5A"
VERSION = 1
FRAME = struct.Struct("<BBBBHI12h4iBB")
CURRENT_SCALE = 1000.0
POSITION_SCALE = 1_000_000.0
VELOCITY_SCALE = 1000.0

SERIES = [
    "m0_a",
    "m0_b",
    "m0_c",
    "m0_d",
    "m0_q",
    "m0_iq_sp",
    "m1_a",
    "m1_b",
    "m1_c",
    "m1_d",
    "m1_q",
    "m1_iq_sp",
    "m0_pos",
    "m1_pos",
    "m0_vel",
    "m1_vel",
]


def default_port():
    ports = sorted(
        glob.glob("/dev/serial/by-id/*")
        + glob.glob("/dev/ttyACM*")
        + glob.glob("/dev/ttyUSB*")
    )
    if not ports:
        raise SystemExit("No serial port found. Pass --port /dev/ttyACM0")
    return ports[0]


def checksum_ok(frame_bytes):
    checksum = 0
    for value in frame_bytes[:-1]:
        checksum ^= value
    return checksum == frame_bytes[-1]


def decode_frame(frame_bytes):
    if len(frame_bytes) != FRAME.size:
        return None
    if frame_bytes[:2] != MAGIC:
        return None
    if frame_bytes[2] != VERSION or frame_bytes[3] != FRAME.size:
        return None
    if not checksum_ok(frame_bytes):
        return None

    unpacked = FRAME.unpack(frame_bytes)
    seq = unpacked[4]
    t_us = unpacked[5]
    currents = unpacked[6:18]
    positions = unpacked[18:22]
    flags = unpacked[22]

    values = {
        "m0_a": currents[0] / CURRENT_SCALE,
        "m0_b": currents[1] / CURRENT_SCALE,
        "m0_c": currents[2] / CURRENT_SCALE,
        "m0_d": currents[3] / CURRENT_SCALE,
        "m0_q": currents[4] / CURRENT_SCALE,
        "m0_iq_sp": currents[5] / CURRENT_SCALE,
        "m1_a": currents[6] / CURRENT_SCALE,
        "m1_b": currents[7] / CURRENT_SCALE,
        "m1_c": currents[8] / CURRENT_SCALE,
        "m1_d": currents[9] / CURRENT_SCALE,
        "m1_q": currents[10] / CURRENT_SCALE,
        "m1_iq_sp": currents[11] / CURRENT_SCALE,
        "m0_pos": positions[0] / POSITION_SCALE,
        "m0_vel": positions[1] / VELOCITY_SCALE,
        "m1_pos": positions[2] / POSITION_SCALE,
        "m1_vel": positions[3] / VELOCITY_SCALE,
    }
    return seq, t_us, flags, values


def capture_binary_telemetry(port, baud, seconds, startup_timeout):
    startup_deadline = time.monotonic() + startup_timeout
    capture_start = None
    device_t0 = None
    last_seq = None
    dropped = 0
    bad_frames = 0
    buffer = bytearray()
    times = []
    data = {name: [] for name in SERIES}

    with serial.Serial(port, baud, timeout=0.02) as ser:
        ser.reset_input_buffer()
        while True:
            now = time.monotonic()
            if capture_start is None:
                if now >= startup_deadline:
                    break
            elif (now - capture_start) >= seconds:
                break

            chunk = ser.read(4096)
            if chunk:
                buffer.extend(chunk)

            while len(buffer) >= FRAME.size:
                magic_at = buffer.find(MAGIC)
                if magic_at < 0:
                    del buffer[:-1]
                    break
                if magic_at:
                    del buffer[:magic_at]
                if len(buffer) < FRAME.size:
                    break

                candidate = bytes(buffer[: FRAME.size])
                decoded = decode_frame(candidate)
                if decoded is None:
                    bad_frames += 1
                    del buffer[0]
                    continue

                del buffer[: FRAME.size]
                seq, t_us, _flags, values = decoded
                if last_seq is not None:
                    dropped += (seq - last_seq - 1) & 0xFFFF
                last_seq = seq

                if device_t0 is None:
                    device_t0 = t_us
                    capture_start = time.monotonic()
                times.append(((t_us - device_t0) & 0xFFFFFFFF) * 1e-6)
                for name in SERIES:
                    data[name].append(values[name])

    return times, data, dropped, bad_frames


def main():
    parser = argparse.ArgumentParser(description="Plot Pico binary motor telemetry.")
    parser.add_argument("seconds", type=float, help="capture duration")
    parser.add_argument("--port", default=None, help="serial port, e.g. /dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--startup-timeout", type=float, default=8.0, help="seconds to wait for the first frame")
    parser.add_argument("--save", help="optional output image path")
    args = parser.parse_args()

    port = args.port or default_port()
    print(f"Reading binary telemetry from {port} at requested {args.baud} baud for {args.seconds:g}s")
    print(f"Frame v{VERSION}: {FRAME.size} bytes, scales: current=mA, position=urad, speed=mrad/s")

    times, data, dropped, bad_frames = capture_binary_telemetry(
        port,
        args.baud,
        args.seconds,
        args.startup_timeout,
    )

    if not times:
        raise SystemExit("No valid binary telemetry frames captured.")

    print(f"Captured {len(times)} samples, dropped_by_sequence={dropped}, bad_frames={bad_frames}")

    fig, axes = plt.subplots(3, 2, sharex=True, figsize=(12, 8))
    fig.suptitle(f"Motor telemetry from {port}")

    m0_current_labels = [
        ("m0_a", "M0 A"),
        ("m0_b", "M0 Bcalc"),
        ("m0_c", "M0 C"),
        ("m0_d", "M0 Id"),
        ("m0_q", "M0 Iq"),
        ("m0_iq_sp", "M0 Iq target"),
    ]
    for name, label in m0_current_labels:
        axes[0, 0].plot(times, data[name], label=label)
    axes[0, 0].set_title("Motor 0")
    axes[0, 0].set_ylabel("Current [A]")
    axes[0, 0].grid(True, alpha=0.3)
    axes[0, 0].legend()

    m1_current_labels = [
        ("m1_a", "M1 A"),
        ("m1_b", "M1 Bcalc"),
        ("m1_c", "M1 C"),
        ("m1_d", "M1 Id"),
        ("m1_q", "M1 Iq"),
        ("m1_iq_sp", "M1 Iq target"),
    ]
    for name, label in m1_current_labels:
        axes[0, 1].plot(times, data[name], label=label)
    axes[0, 1].set_title("Motor 1")
    axes[0, 1].grid(True, alpha=0.3)
    axes[0, 1].legend()

    axes[1, 0].plot(times, data["m0_pos"], label="M0 encoder position")
    axes[1, 0].set_ylabel("Position [rad]")
    axes[1, 0].grid(True, alpha=0.3)
    axes[1, 0].legend()

    axes[1, 1].plot(times, data["m1_pos"], label="M1 encoder position")
    axes[1, 1].grid(True, alpha=0.3)
    axes[1, 1].legend()

    axes[2, 0].plot(times, data["m0_vel"], label="M0 encoder speed")
    axes[2, 0].set_xlabel("Time [s]")
    axes[2, 0].set_ylabel("Speed [rad/s]")
    axes[2, 0].grid(True, alpha=0.3)
    axes[2, 0].legend()

    axes[2, 1].plot(times, data["m1_vel"], label="M1 encoder speed")
    axes[2, 1].set_xlabel("Time [s]")
    axes[2, 1].grid(True, alpha=0.3)
    axes[2, 1].legend()

    fig.tight_layout()

    if args.save:
        fig.savefig(args.save, dpi=150)
    else:
        plt.show()


if __name__ == "__main__":
    main()
