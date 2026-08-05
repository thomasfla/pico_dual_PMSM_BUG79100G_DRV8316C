#!/usr/bin/env python3
import argparse
import time

import matplotlib.pyplot as plt
import serial

from usb_motor_protocol import STATE_FRAME, STATE_VALUE_NAMES, default_port, pop_state_frames


SERIES = STATE_VALUE_NAMES + ("control_loop_us", "latest_command_index")


def capture_state_packets(port, baud, seconds, startup_timeout):
    startup_deadline = time.monotonic() + startup_timeout
    capture_start = None
    device_t0 = None
    last_seq = None
    dropped = 0
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

            for state in pop_state_frames(buffer):
                seq = state["sequence"]
                if last_seq is not None:
                    dropped += (seq - last_seq - 1) & 0xFFFF
                last_seq = seq

                if device_t0 is None:
                    device_t0 = state["t_us"]
                    capture_start = time.monotonic()
                times.append(((state["t_us"] - device_t0) & 0xFFFFFFFF) * 1e-6)
                for name in SERIES:
                    data[name].append(state[name])

    return times, data, dropped


def plot(times, data, port, save_path):
    fig, axes = plt.subplots(4, 2, sharex=True, figsize=(12, 10))
    fig.suptitle(f"Motor state from {port}")

    for col, motor in enumerate(("m0", "m1")):
        axes[0, col].set_title("Motor 0" if motor == "m0" else "Motor 1")
        axes[0, col].plot(times, data[f"{motor}_i"], label="Iq")
        axes[0, col].plot(times, data[f"{motor}_i_target"], label="Iq target")
        axes[0, col].set_ylabel("Current [A]")
        axes[1, col].plot(times, data[f"{motor}_q"], label="q")
        axes[1, col].set_ylabel("Position [rad]")
        axes[2, col].plot(times, data[f"{motor}_v"], label="v")
        axes[2, col].set_ylabel("Speed [rad/s]")
        axes[3, col].plot(times, data["control_loop_us"], label="Control loop")
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
    parser = argparse.ArgumentParser(description="Plot Pico USB motor state packets.")
    parser.add_argument("seconds", type=float, help="capture duration")
    parser.add_argument("--port", default=None, help="serial port, e.g. /dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--startup-timeout", type=float, default=8.0)
    parser.add_argument("--save", help="optional output image path")
    args = parser.parse_args()

    port = args.port or default_port()
    print(f"Reading state packets from {port}: {STATE_FRAME.size} bytes/frame")
    times, data, dropped = capture_state_packets(
        port, args.baud, args.seconds, args.startup_timeout
    )
    if not times:
        raise SystemExit("No valid state packets captured.")

    print(f"Captured {len(times)} samples, dropped_by_sequence={dropped}")
    plot(times, data, port, args.save)


if __name__ == "__main__":
    main()
