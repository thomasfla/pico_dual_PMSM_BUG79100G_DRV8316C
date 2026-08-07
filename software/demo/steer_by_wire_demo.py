#!/usr/bin/env python3
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))

from motor_usb_client import MotorUsbController

KP = 0.5
KD = 0.05


with MotorUsbController(timeout_ms=20) as robot:
    robot.initialize()
    q0_start = robot.m0.q
    q1_start = robot.m1.q

    while True:
        robot.m0.set(q=q0_start + robot.m1.q - q1_start, v=0.8*robot.m1.v, kp=KP, kd=KD)
        robot.m1.set(q=q1_start + robot.m0.q - q0_start, v=0.8*robot.m0.v, kp=KP, kd=KD)
        robot.update()
