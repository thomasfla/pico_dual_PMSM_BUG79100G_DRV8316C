import time
from dataclasses import dataclass
from threading import Condition, Thread

import serial

from usb_motor_protocol import (
    BOTH_MOTORS,
    M0_READY,
    M1_READY,
    default_port,
    encode_command,
    pop_state_frames,
)


DEFAULT_MAX_COMMAND_RATE_HZ = 1000.0


@dataclass
class Motor:
    q: float = 0.0
    v: float = 0.0
    i: float = 0.0
    i_target: float = 0.0
    kp: float = 0.0
    kd: float = 0.0
    iff: float = 0.0
    q_target: float = 0.0
    v_target: float = 0.0
    enabled: bool = False

    def set(self, q=None, v=None, kp=None, kd=None, iff=None, enabled=True):
        if q is not None:
            self.q_target = q
        if v is not None:
            self.v_target = v
        if kp is not None:
            self.kp = kp
        if kd is not None:
            self.kd = kd
        if iff is not None:
            self.iff = iff
        self.enabled = enabled

    def packet_values(self):
        return (self.kp, self.kd, self.iff, self.q_target, self.v_target)

    def update_state(self, q, v, i, i_target):
        self.q = q
        self.v = v
        self.i = i
        self.i_target = i_target


class MotorUsbController:
    def __init__(
        self,
        port=None,
        baud=115200,
        timeout_ms=20,
        max_command_rate_hz=DEFAULT_MAX_COMMAND_RATE_HZ,
    ):
        self.port = port or default_port()
        self.baud = baud
        self.timeout_ms = timeout_ms
        self.min_command_period_s = (
            1.0 / max_command_rate_hz if max_command_rate_hz > 0.0 else 0.0
        )
        self.m0 = Motor()
        self.m1 = Motor()
        self.state = {}
        self.latency_ms = None
        self.last_rate_sleep_ms = 0.0
        self.last_write_ms = 0.0
        self.last_update_ms = 0.0
        self.state_drops = 0
        self.command_index = 1
        self._buffer = bytearray()
        self._send_times = {}
        self._last_state_sequence = None
        self._last_send_time = 0.0
        self._serial = None
        self._condition = Condition()
        self._rx_running = False
        self._rx_thread = None

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, _exc_type, _exc, _tb):
        self.close()

    def open(self):
        self._serial = serial.Serial(self.port, self.baud, timeout=0.001)
        self._serial.reset_input_buffer()
        self._rx_running = True
        self._rx_thread = Thread(target=self._read_loop, daemon=True)
        self._rx_thread.start()

    def close(self):
        if self._serial is not None:
            self.zero_torque(block=False)
            self._rx_running = False
            if self._rx_thread is not None:
                self._rx_thread.join(timeout=0.2)
            self._serial.close()
            self._serial = None
            self._rx_thread = None

    @property
    def ready_flags(self):
        with self._condition:
            return self.state.get("flags", 0)

    def initialize(self, timeout_s=2.0, flags=BOTH_MOTORS):
        self.m0.set(
            q=0.0, v=0.0, kp=0.0, kd=0.0, iff=0.0, enabled=bool(flags & M0_READY)
        )
        self.m1.set(
            q=0.0, v=0.0, kp=0.0, kd=0.0, iff=0.0, enabled=bool(flags & M1_READY)
        )
        return self.update(timeout_ms=0, block=True, timeout_s=timeout_s)

    def zero_torque(self, block=True):
        self.m0.set(kp=0.0, kd=0.0, iff=0.0, enabled=bool(self.ready_flags & M0_READY))
        self.m1.set(kp=0.0, kd=0.0, iff=0.0, enabled=bool(self.ready_flags & M1_READY))
        return self.update(timeout_ms=0, block=block)

    def update(self, timeout_ms=None, block=False, wait_state=False, timeout_s=0.05):
        update_start = time.perf_counter()
        index = self.command_index
        previous_sequence = self.poll().get("sequence")
        packet = encode_command(
            index,
            self._command_flags(),
            self.timeout_ms if timeout_ms is None else timeout_ms,
            self.m0.packet_values(),
            self.m1.packet_values(),
        )
        rate_sleep_s = self._limit_command_rate()
        sent_at = time.monotonic()
        with self._condition:
            self._send_times[index] = sent_at
            while len(self._send_times) > 512:
                self._send_times.pop(next(iter(self._send_times)))
        write_start = time.perf_counter()
        self._serial.write(packet)
        write_end = time.perf_counter()
        self.last_rate_sleep_ms = rate_sleep_s * 1000.0
        self.last_write_ms = (write_end - write_start) * 1000.0
        self.last_update_ms = (time.perf_counter() - update_start) * 1000.0
        self.command_index = (index + 1) & 0xFFFFFFFF or 1
        if block:
            return self.wait_for_echo(index, timeout_s)
        if wait_state:
            return self.wait_for_state(previous_sequence, timeout_s)
        return self.poll()

    def poll(self):
        with self._condition:
            return dict(self.state)

    def wait_for_echo(self, command_index, timeout_s=0.05):
        deadline = time.monotonic() + timeout_s
        with self._condition:
            while True:
                state = dict(self.state)
                if state.get("latest_command_index") == command_index:
                    return state
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    break
                self._condition.wait(remaining)
        raise TimeoutError(f"Command {command_index} was not echoed by core1")

    def wait_for_state(self, previous_sequence=None, timeout_s=0.05):
        deadline = time.monotonic() + timeout_s
        with self._condition:
            while True:
                state = dict(self.state)
                sequence = state.get("sequence")
                if state and (previous_sequence is None or sequence != previous_sequence):
                    return state
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    break
                self._condition.wait(remaining)
        raise TimeoutError("No new state packet received")

    def _read_loop(self):
        while self._rx_running:
            chunk = self._serial.read(self._serial.in_waiting or 1)
            if not chunk:
                continue
            self._buffer.extend(chunk)
            for state in pop_state_frames(self._buffer):
                with self._condition:
                    if self._last_state_sequence is not None:
                        self.state_drops += (
                            state["sequence"] - self._last_state_sequence - 1
                        ) & 0xFFFF
                    self._last_state_sequence = state["sequence"]
                    self.state = state
                    self.m0.update_state(
                        state["m0_q"],
                        state["m0_v"],
                        state["m0_i"],
                        state["m0_i_target"],
                    )
                    self.m1.update_state(
                        state["m1_q"],
                        state["m1_v"],
                        state["m1_i"],
                        state["m1_i_target"],
                    )
                    echoed = state["latest_command_index"]
                    sent_at = self._send_times.pop(echoed, None)
                    if sent_at is not None:
                        self.latency_ms = (time.monotonic() - sent_at) * 1000.0
                    self._condition.notify_all()

    def _command_flags(self):
        flags = 0
        if self.m0.enabled:
            flags |= M0_READY
        if self.m1.enabled:
            flags |= M1_READY
        return flags

    def _limit_command_rate(self):
        now = time.monotonic()
        sleep_s = self.min_command_period_s - (now - self._last_send_time)
        if sleep_s > 0.0:
            time.sleep(sleep_s)
            now = time.monotonic()
        else:
            sleep_s = 0.0
        self._last_send_time = now
        return sleep_s
