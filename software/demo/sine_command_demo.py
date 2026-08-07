#!/usr/bin/env python3
import math
from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))

from motor_usb_client import MotorUsbController

amplitude = 3.0
frequency = 1.0
with MotorUsbController(timeout_ms=20) as robot:
    robot.initialize()
    q0, q1 = robot.m0.q, robot.m1.q
    t0 = time.monotonic()
    while time.monotonic() - t0 < 10.0:
        t = time.monotonic() - t0
        q = amplitude * math.sin(2.0 * math.pi * frequency * t)
        v = amplitude * 2.0 * math.pi * frequency * math.cos(2.0 * math.pi * frequency * t)
        robot.m0.set(q=q0 + q, v=v, kp=1.0, kd=0.03)
        robot.m1.set(q=q1 + q, v=v, kp=1.0, kd=0.03)
        robot.update()
