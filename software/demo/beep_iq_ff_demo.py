#!/usr/bin/env python3
import argparse
from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))

from motor_usb_client import MotorUsbController


def parse_args():
    parser = argparse.ArgumentParser(
        description="Make the motor emit a tone with square-wave Iq feedforward current."
    )
    parser.add_argument("--motor", choices=("0", "1", "both"), default="both")
    parser.add_argument("--current", type=float, default=0.2, help="Iq_ff amplitude [A]")
    parser.add_argument("--frequency", type=float, default=220.0, help="Tone frequency [Hz]")
    parser.add_argument("--seconds", type=float, default=0.5, help="Tone duration [s]")
    parser.add_argument("--rate", type=float, default=2000.0, help="USB command rate [Hz]")
    parser.add_argument("--timeout-ms", type=int, default=50)
    return parser.parse_args()


def set_iq(robot, motor, iq):
    robot.m0.set(kp=0.0, kd=0.0, iff=iq if motor in ("0", "both") else 0.0,
                 enabled=motor in ("0", "both"))
    robot.m1.set(kp=0.0, kd=0.0, iff=iq if motor in ("1", "both") else 0.0,
                 enabled=motor in ("1", "both"))


def main():
    args = parse_args()
    period_s = 1.0 / args.rate

    with MotorUsbController(
        timeout_ms=args.timeout_ms,
        max_command_rate_hz=args.rate,
    ) as robot:
        robot.initialize()

        t0 = time.monotonic()
        next_tick = time.perf_counter()
        try:
            while True:
                t = time.monotonic() - t0
                if t >= args.seconds:
                    break

                iq = args.current if (t * args.frequency) % 1.0 < 0.5 else -args.current
                set_iq(robot, args.motor, iq)
                robot.update()

                next_tick += period_s
                sleep_s = next_tick - time.perf_counter()
                if sleep_s > 0.0:
                    time.sleep(sleep_s)
                else:
                    next_tick = time.perf_counter()
        finally:
            robot.zero_torque(block=False)


if __name__ == "__main__":
    main()
