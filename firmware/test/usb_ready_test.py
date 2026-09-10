"""Host readiness checks, using real packet decoding and a fake command echo."""
import unittest
from unittest.mock import Mock

from motor_usb import MotorUsbController
from motor_usb.protocol import (
    BOTH_MOTORS, M0_READY, CONTROL_FAULT, STATE_FRAME, checksum,
    decode_state, require_ready,
)


def state_packet(flags):
    frame = STATE_FRAME.pack(0xA5, 0x5A, 0x53, 2, 61, 1, 1000, 1, 50.0, *([0.0] * 10), flags, 0)
    return decode_state(frame[:-1] + bytes([checksum(frame)]))


class ReadinessTest(unittest.TestCase):
    def test_ready(self):
        require_ready(state_packet(BOTH_MOTORS))
        require_ready(state_packet(M0_READY), M0_READY)

    def test_original_silent_fault(self):
        with self.assertRaisesRegex(RuntimeError, "older firmware.*flags=0x04"):
            require_ready(state_packet(CONTROL_FAULT))

    def test_fault_reason(self):
        with self.assertRaisesRegex(RuntimeError, "PWM output deadline missed.*flags=0x44"):
            require_ready(state_packet(CONTROL_FAULT | (8 << 3)))

    def test_missing_motor(self):
        with self.assertRaisesRegex(RuntimeError, "not ready: M1"):
            require_ready(state_packet(M0_READY))

    def test_emergency_stop_reset_instructions(self):
        with self.assertRaisesRegex(RuntimeError, "E-stop.*flags=0x4c.*3 seconds"):
            require_ready(state_packet(CONTROL_FAULT | (9 << 3)))

    def test_initialize_checks_echo_readiness(self):
        robot = MotorUsbController(port="unused")
        robot.update = Mock(return_value=state_packet(CONTROL_FAULT))
        with self.assertRaisesRegex(RuntimeError, "Controller fault"):
            robot.initialize()
        robot.update.assert_called_once_with(timeout_ms=0, block=True, timeout_s=2.0)
        self.assertEqual(robot.m0.kp, 0)
        self.assertEqual(robot.m1.iff, 0)

    def test_runtime_check(self):
        robot = MotorUsbController(port="unused")
        robot.state = state_packet(BOTH_MOTORS)
        robot.check_ready()
        robot.state = state_packet(CONTROL_FAULT | (3 << 3))
        with self.assertRaisesRegex(RuntimeError, "ADC frame skipped"):
            robot.check_ready()


if __name__ == "__main__":
    unittest.main()
