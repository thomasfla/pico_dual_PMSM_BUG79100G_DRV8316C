#!/usr/bin/env python3
import argparse
import glob
import struct
import time

import matplotlib.pyplot as plt
import serial


MAGIC = b"\xA5\x5A"
VERSION = 3
FRAME = struct.Struct("<BBBBHIf18fBB")
VALUE_NAMES = (
    "m0_pos", "m0_vel", "m0_q", "m0_iq_sp", "m0_pos_sp", "m0_vel_sp", "m0_ff", "m0_kp", "m0_kd",
    "m1_pos", "m1_vel", "m1_q", "m1_iq_sp", "m1_pos_sp", "m1_vel_sp", "m1_ff", "m1_kp", "m1_kd",
)
SERIES = VALUE_NAMES + ("ctrl_us",)


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
    if frame_bytes[:2] != MAGIC or frame_bytes[2] != VERSION or frame_bytes[3] != FRAME.size:
        return None
    if not checksum_ok(frame_bytes):
        return None

    unpacked = FRAME.unpack(frame_bytes)
    seq = unpacked[4]
    t_us = unpacked[5]
    ctrl_us = unpacked[6]
    values = dict(zip(VALUE_NAMES, unpacked[7:25]))
    values["ctrl_us"] = ctrl_us
    flags = unpacked[25]
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

                candidate = bytes(buffer[:FRAME.size])
                decoded = decode_frame(candidate)
                if decoded is None:
                    bad_frames += 1
                    del buffer[0]
                    continue

                del buffer[:FRAME.size]
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


def plot(times, data, port, save_path):
    fig, axes = plt.subplots(4, 2, sharex=True, figsize=(12, 10))
    fig.suptitle(f"Motor telemetry from {port}")

    for col, motor in enumerate(("m0", "m1")):
        title = "Motor 0" if motor == "m0" else "Motor 1"
        axes[0, col].set_title(title)
        axes[0, col].plot(times, data[f"{motor}_q"], label="Iq")
        axes[0, col].plot(times, data[f"{motor}_iq_sp"], label="Iq target")
        axes[0, col].set_ylabel("Current [A]")
        axes[1, col].plot(times, data[f"{motor}_pos"], label="Position")
        axes[1, col].plot(times, data[f"{motor}_pos_sp"], label="Target")
        axes[1, col].set_ylabel("Position [rad]")
        axes[2, col].plot(times, data[f"{motor}_vel"], label="Velocity")
        axes[2, col].plot(times, data[f"{motor}_vel_sp"], label="Target")
        axes[2, col].set_ylabel("Speed [rad/s]")
        axes[3, col].plot(times, data["ctrl_us"], label="Control loop")
        axes[3, col].set_ylabel("Duration [us]")
        axes[3, col].set_xlabel("Time [s]")
        for row in range(4):
            axes[row, col].grid(True, alpha=0.3)
            axes[row, col].legend()

    fig.tight_layout()
    if save_path:
        fig.savefig(save_path, dpi=150)
    else:
        plt.show()


def main():
    parser = argparse.ArgumentParser(description="Plot Pico v3 binary motor telemetry.")
    parser.add_argument("seconds", type=float, help="capture duration")
    parser.add_argument("--port", default=None, help="serial port, e.g. /dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--startup-timeout", type=float, default=8.0)
    parser.add_argument("--save", help="optional output image path")
    args = parser.parse_args()

    port = args.port or default_port()
    print(f"Reading v{VERSION} telemetry from {port}: {FRAME.size} bytes/frame")
    times, data, dropped, bad_frames = capture_binary_telemetry(
        port, args.baud, args.seconds, args.startup_timeout
    )
    if not times:
        raise SystemExit("No valid binary telemetry frames captured.")

    print(f"Captured {len(times)} samples, dropped_by_sequence={dropped}, bad_frames={bad_frames}")
    plot(times, data, port, args.save)


if __name__ == "__main__":
    main()
