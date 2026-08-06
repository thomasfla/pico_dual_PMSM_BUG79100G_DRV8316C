#!/usr/bin/env python3
from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))

from motor_usb_client import MotorUsbController


RATE_HZ = 50.0
PERIOD_S = 1.0 / RATE_HZ
TIMEOUT_MS = 100


with MotorUsbController(timeout_ms=TIMEOUT_MS) as robot:
    robot.initialize()
    next_tick = time.perf_counter()

    while True:
        robot.m0.set(q=robot.m0.q, v=0.0, kp=0.0, kd=0.0, iff=0.0)
        robot.m1.set(q=robot.m1.q, v=0.0, kp=0.0, kd=0.0, iff=0.0)
        robot.update()

        print(
            f"t={robot.state.get('t_us', 0) * 1e-6:.3f} "
            f"loop_us={robot.state.get('control_loop_us', 0.0):.2f} "
            f"M0 q={robot.m0.q:.5f} v={robot.m0.v:.3f} "
            f"i={robot.m0.i:.3f} i_tgt={robot.m0.i_target:.3f} "
            f"M1 q={robot.m1.q:.5f} v={robot.m1.v:.3f} "
            f"i={robot.m1.i:.3f} i_tgt={robot.m1.i_target:.3f}"
        )

        next_tick += PERIOD_S
        sleep_s = next_tick - time.perf_counter()
        if sleep_s > 0.0:
            time.sleep(sleep_s)
        else:
            next_tick = time.perf_counter()
