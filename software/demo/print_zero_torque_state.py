#!/usr/bin/env python3
from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))

from motor_usb_client import MotorUsbController


RATE_HZ = 50.0
PERIOD_S = 1.0 / RATE_HZ
TIMEOUT_MS = 100
CURSOR_HOME = "\033[H"
CLEAR_SCREEN = "\033[2J"
ERASE_TO_END = "\033[J"


def format_screen(robot):
    state = robot.state
    return (
        f"{CURSOR_HOME}{ERASE_TO_END}"
        "Pico dual PMSM state - zero torque command at 50 Hz\n"
        "\n"
        f"{'t [s]':>12} {'loop [us]':>12} {'cmd echo':>12} {'drops':>8}\n"
        f"{state.get('t_us', 0) * 1e-6:12.3f} "
        f"{state.get('control_loop_us', 0.0):12.2f} "
        f"{state.get('latest_command_index', 0):12d} "
        f"{robot.state_drops:8d}\n"
        "\n"
        f"{'motor':<6} {'q [rad]':>12} {'v [rad/s]':>12} "
        f"{'Iq [A]':>12} {'Iq target [A]':>14}\n"
        f"{'M0':<6} {robot.m0.q:12.5f} {robot.m0.v:12.3f} "
        f"{robot.m0.i:12.3f} {robot.m0.i_target:14.3f}\n"
        f"{'M1':<6} {robot.m1.q:12.5f} {robot.m1.v:12.3f} "
        f"{robot.m1.i:12.3f} {robot.m1.i_target:14.3f}\n"
        "\n"
        "Ctrl-C to stop.\n"
    )


with MotorUsbController(timeout_ms=TIMEOUT_MS) as robot:
    robot.initialize()
    next_tick = time.perf_counter()
    sys.stdout.write(CLEAR_SCREEN)

    while True:
        robot.m0.set(q=robot.m0.q, v=0.0, kp=0.0, kd=0.0, iff=0.0)
        robot.m1.set(q=robot.m1.q, v=0.0, kp=0.0, kd=0.0, iff=0.0)
        robot.update()

        sys.stdout.write(format_screen(robot))
        sys.stdout.flush()

        next_tick += PERIOD_S
        sleep_s = next_tick - time.perf_counter()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.perf_counter()
