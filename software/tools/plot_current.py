#!/usr/bin/env python3
import argparse
import time

import matplotlib.pyplot as plt

from motor_usb_client import MotorUsbController
from usb_motor_protocol import STATE_VALUE_NAMES, default_port


SERIES = STATE_VALUE_NAMES + (
    "m0_i_cmd",
    "m1_i_cmd",
    "control_loop_us",
    "latest_command_index",
)


def set_square_wave_iq(robot, motor, iq):
    m0_active = motor in ("0", "both")
    m1_active = motor in ("1", "both")
    robot.m0.set(kp=0.0, kd=0.0, iff=iq if m0_active else 0.0, enabled=m0_active)
    robot.m1.set(kp=0.0, kd=0.0, iff=iq if m1_active else 0.0, enabled=m1_active)


def capture_square_wave(
    port,
    baud,
    seconds,
    startup_timeout,
    motor,
    amplitude,
    frequency,
    command_rate,
    timeout_ms,
):
    command_period_s = 1.0 / command_rate
    t0 = None
    device_t0 = None
    last_seq = None
    dropped = 0
    times = []
    data = {name: [] for name in SERIES}

    with MotorUsbController(
        port=port,
        baud=baud,
        timeout_ms=timeout_ms,
        max_command_rate_hz=command_rate,
    ) as robot:
        robot.initialize(timeout_s=startup_timeout)
        t0 = time.monotonic()
        next_tick = time.perf_counter()

        while True:
            elapsed = time.monotonic() - t0
            if elapsed >= seconds:
                break

            iq = amplitude if (elapsed * frequency) % 1.0 < 0.5 else -amplitude
            set_square_wave_iq(robot, motor, iq)
            state = robot.update(block=False)
            seq = state.get("sequence")
            if seq is not None and seq != last_seq:
                if last_seq is not None:
                    dropped += (seq - last_seq - 1) & 0xFFFF
                last_seq = seq

                if device_t0 is None:
                    device_t0 = state["t_us"]
                times.append(((state["t_us"] - device_t0) & 0xFFFFFFFF) * 1e-6)
                for name in SERIES:
                    if name == "m0_i_cmd":
                        data[name].append(iq if motor in ("0", "both") else 0.0)
                    elif name == "m1_i_cmd":
                        data[name].append(iq if motor in ("1", "both") else 0.0)
                    else:
                        data[name].append(state[name])

            next_tick += command_period_s
            sleep_s = next_tick - time.perf_counter()
            if sleep_s > 0.0:
                time.sleep(sleep_s)
            else:
                next_tick = time.perf_counter()

        robot.zero_torque(block=False)

    return times, data, dropped


def plot(times, data, port, save_path, motor, amplitude, frequency):
    fig, axes = plt.subplots(4, 2, sharex=True, figsize=(12, 10))
    fig.suptitle(
        f"Current loop square-wave test from {port}: "
        f"motor={motor}, amplitude={amplitude:.3f} A, frequency={frequency:.2f} Hz"
    )

    for col, motor in enumerate(("m0", "m1")):
        axes[0, col].set_title("Motor 0" if motor == "m0" else "Motor 1")
        axes[0, col].plot(times, data[f"{motor}_i"], label="Iq")
        axes[0, col].plot(times, data[f"{motor}_i_target"], label="Iq target")
        axes[0, col].plot(times, data[f"{motor}_i_cmd"], "--", label="Iq command")
        axes[0, col].set_ylabel("Current [A]")
        axes[1, col].plot(times, data[f"{motor}_q"], label="q")
        axes[1, col].set_ylabel("Position [rad]")
        axes[2, col].plot(times, data[f"{motor}_v"], label="v filtered")
        axes[2, col].plot(
            times,
            data[f"{motor}_v_highfrequency"],
            label="v_highfrequency",
            alpha=0.75,
        )
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
    parser = argparse.ArgumentParser(
        description="Command a small square-wave Iq target and plot current-loop response."
    )
    parser.add_argument("seconds", type=float, help="capture duration")
    parser.add_argument("--port", default=None, help="serial port, e.g. /dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--startup-timeout", type=float, default=8.0)
    parser.add_argument("--motor", choices=("0", "1", "both"), default="both")
    parser.add_argument("--amplitude", type=float, default=0.2, help="Iq amplitude [A]")
    parser.add_argument("--frequency", type=float, default=2.0, help="square-wave frequency [Hz]")
    parser.add_argument("--rate", type=float, default=1000.0, help="USB command rate [Hz]")
    parser.add_argument("--timeout-ms", type=int, default=50)
    parser.add_argument("--save", help="optional output image path")
    args = parser.parse_args()
    if args.seconds <= 0.0:
        raise SystemExit("seconds must be > 0")
    if args.amplitude <= 0.0:
        raise SystemExit("--amplitude must be > 0")
    if args.frequency <= 0.0:
        raise SystemExit("--frequency must be > 0")
    if args.rate <= 0.0:
        raise SystemExit("--rate must be > 0")

    port = args.port or default_port()
    print(
        f"Commanding {args.amplitude:.3f} A square-wave Iq at {args.frequency:.2f} Hz "
        f"on motor {args.motor} for {args.seconds:.2f} s from {port}"
    )
    times, data, dropped = capture_square_wave(
        port,
        args.baud,
        args.seconds,
        args.startup_timeout,
        args.motor,
        args.amplitude,
        args.frequency,
        args.rate,
        args.timeout_ms,
    )
    if not times:
        raise SystemExit("No valid state packets captured.")

    print(f"Captured {len(times)} samples, dropped_by_sequence={dropped}")
    plot(times, data, port, args.save, args.motor, args.amplitude, args.frequency)


if __name__ == "__main__":
    main()
