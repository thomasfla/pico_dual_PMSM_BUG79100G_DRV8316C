# USB Motor Control Protocol

Binary packets are sent over USB CDC. The baud rate value is ignored by USB CDC, but host tools use `115200` as the serial line coding.

All multi-byte values are little-endian. Packets are packed with no padding. The checksum is the XOR of every packet byte except the final checksum byte.

## Common Header

| Offset | Type | Field | Value |
| ---: | --- | --- | --- |
| 0 | `uint8` | `magic0` | `0xA5` |
| 1 | `uint8` | `magic1` | `0x5A` |
| 2 | `uint8` | `type` | `'C'` command or `'S'` state |
| 3 | `uint8` | `version` | `2` |
| 4 | `uint8` | `length` | packet length in bytes |

## Command Packet, PC To Board

Type `'C'`, length `53` bytes.

Format string used by the Python helper: `<BBBBBIBH10fB`.

| Offset | Type | Field | Unit |
| ---: | --- | --- | --- |
| 0 | `uint8` | `magic0` | |
| 1 | `uint8` | `magic1` | |
| 2 | `uint8` | `type = 'C'` | |
| 3 | `uint8` | `version` | |
| 4 | `uint8` | `length = 53` | |
| 5 | `uint32` | `command_index` | arbitrary host sequence |
| 9 | `uint8` | `flags` | bit0 M0 command valid, bit1 M1 command valid |
| 10 | `uint16` | `timeout_ms` | watchdog timeout |
| 12 | `float32` | `m0_kp` | A/rad |
| 16 | `float32` | `m0_kd` | A/(rad/s) |
| 20 | `float32` | `m0_iff` | A |
| 24 | `float32` | `m0_q_target` | rad |
| 28 | `float32` | `m0_v_target` | rad/s |
| 32 | `float32` | `m1_kp` | A/rad |
| 36 | `float32` | `m1_kd` | A/(rad/s) |
| 40 | `float32` | `m1_iff` | A |
| 44 | `float32` | `m1_q_target` | rad |
| 48 | `float32` | `m1_v_target` | rad/s |
| 52 | `uint8` | `checksum` | XOR |

The `flags` byte selects which motor commands are active. A motor without its command-valid bit set is commanded to zero Iq, even if the robot has consumed a recent command.

`timeout_ms = 0` disables or resets the command watchdog. `timeout_ms > 0` enables it. The timeout runs from receipt of a valid packet on core0, so a command queued during startup cannot obtain a new lease when core1 eventually consumes it. On expiry, both motors are commanded to zero Iq until another unexpired command is consumed. Disabling this watchdog does not disable feedback or driver-fault protection.

At firmware startup, no command has been consumed, so torque is disabled even though the motors may be initialized.

Normal boot requires a stored (or explicitly configured) sensor direction and electrical zero for each motor. Missing calibration leaves that motor's ready bit clear; normal boot never performs powered alignment. Enter the `!` calibration wizard to align and save it. Command indices are still acknowledged when motors are unavailable.

## State Packet, Board To PC

Type `'S'`, length `61` bytes.

Format string used by the Python helper: `<BBBBBHIIf10fBB`.

| Offset | Type | Field | Unit |
| ---: | --- | --- | --- |
| 0 | `uint8` | `magic0` | |
| 1 | `uint8` | `magic1` | |
| 2 | `uint8` | `type = 'S'` | |
| 3 | `uint8` | `version` | |
| 4 | `uint8` | `length = 61` | |
| 5 | `uint16` | `sequence` | board state sequence |
| 7 | `uint32` | `t_us` | us |
| 11 | `uint32` | `latest_command_index` | last command consumed by core1 |
| 15 | `float32` | `control_loop_us` | us |
| 19 | `float32` | `m0_q` | rad |
| 23 | `float32` | `m0_v` | rad/s, PC-facing filtered velocity |
| 27 | `float32` | `m0_v_highfrequency` | rad/s, internal PD velocity |
| 31 | `float32` | `m0_i` | A |
| 35 | `float32` | `m0_i_target` | A |
| 39 | `float32` | `m1_q` | rad |
| 43 | `float32` | `m1_v` | rad/s, PC-facing filtered velocity |
| 47 | `float32` | `m1_v_highfrequency` | rad/s, internal PD velocity |
| 51 | `float32` | `m1_i` | A |
| 55 | `float32` | `m1_i_target` | A |
| 59 | `uint8` | `flags` | bit0 M0 ready, bit1 M1 ready, bit2 latched control fault, bits3–7 fault cause |
| 60 | `uint8` | `checksum` | XOR |

`m*_v_highfrequency` is the velocity used by the onboard PD loop. `m*_v`
is additionally low-pass filtered for external PC controllers. The firmware
uses a 50 Hz telemetry velocity cutoff, chosen as one tenth of the minimum
planned 500 Hz PC control rate. The equivalent first-order time constant is
`1 / (2*pi*50) = 3.18 ms`.

`latest_command_index` is copied only on core1 when the motor control loop consumes a command. The PC can measure command path latency by recording send time for a command index and waiting until a state packet echoes the same index.

The FOC loop consumes one shared ADC frame per 20 kHz PWM period. At frame N,
it commits the outputs computed from frame N-1, then computes frame N's
outputs. This gives a full period for both motors' calculations and an
approximately 75 us sample-to-output delay. `control_loop_us` reports the
average interval between executed frames (approximately 50 us), not CPU
execution time. With a PWM deadline fault (cause 8), it reports the last
measured control pass duration to aid diagnosis; otherwise it becomes zero
when no frames execute. Velocity uses a fixed 1 ms observation window.

Loss of fresh current feedback for more than 250 us, encoder read failures lasting more than 500 us, assertion of `nFAULT`, invalid current samples, or a missed control/PWM deadline latch a fault. Both drivers are stopped and switched off, both ready bits clear, and bit2 is set. Telemetry and command acknowledgements continue. A board reset is required to leave this state; USB commands cannot re-enable the drivers.

When bit2 is set, `flags >> 3` reports the first fault cause:

| Code | Cause |
| ---: | --- |
| 0 | Unspecified (older firmware) |
| 1 | Driver `nFAULT` |
| 2 | Invalid or stale current feedback |
| 3 | Skipped ADC frame |
| 4 | ADC frame observed outside the PWM control half-cycle |
| 5 / 6 | Stale M0 / M1 encoder feedback |
| 7 | Non-finite control result |
| 8 | PWM output deadline missed |
| 9 | GPIO 28 emergency stop (`flags = 0x4c`) |

GPIO 28 is an active-low input with an internal pull-up. It is polled; no GPIO
interrupt is registered. An observed press immediately inhibits the six PWM
inputs, then the control core switches both drivers off and reports cause 9.
Holding the initial stop press never clears the fault.

To reset an emergency stop:

1. Release the button; the high level must remain stable for 20 ms.
2. Press it again and hold continuously for 3 seconds. Recovery starts at that
   threshold without waiting for release.

Continuing to hold the reset press does not trigger another stop. Release and
press again to stop the motors again.

The firmware checks driver status, fresh current data and the previously ready
motors' encoders before restoring readiness. Motors remain inhibited throughout
the hold and the checks. Integrators and current targets are reset, and commands
received while stopped are discarded. Send a new command after recovery to
resume motion. Calibration and shaft turn tracking are retained; no reboot or
alignment is performed. Other fault causes still require a board reset.

The same polling inhibit protects calibration and board-test operation. Core1
polls during blocking boot tools; reset is serviced when the tool returns to its
serial prompt. An interrupted powered operation must be started again.

Frame observation is allowed during the descending PWM half-cycle. The compare
write of the previous frame checks that the counter is still descending and at
least 3 us remain before zero. Observing a frame more than 4 us after the peak
is allowed. Computation of the next outputs must finish within 47 us, and a
skipped ADC frame still latches a fault.

Mechanical zero is stored as a signed single-turn sensor offset. Legacy offsets containing full turns are normalized when applied. At boot, the reported position starts on the revolution nearest mechanical zero (within +/- pi), then tracks turns continuously until reset. An absolute multi-turn position across power cycles is not available from this encoder.

## Recommended Startup Sequence

1. Open the USB CDC port.
2. Send a command with zero gains, zero feedforward, zero targets, and `timeout_ms = 0`.
3. Wait until `latest_command_index` in a state packet equals that command index.
4. Check the required motor ready bits and ensure the fault bit is clear, then start the real command loop with a finite timeout, for example `timeout_ms = 20`.

The included `../software/demo_sine_position.py` relies on the library to check
ready/fault bits during initialization and each update. It reports the fault
cause instead of continuing silently with disabled motors.

## Boot Calibration Mode

If the first non-newline byte received on USB CDC during the boot entry window is `!`, the firmware does not start the binary protocol. Core0 runs the text-based calibration wizard while core1 polls the emergency-stop button. The entry window is configured by `CALIBRATION_ENTRY_WAIT_MS`.

The wizard can run SimpleFOC electrical angle calibration, capture mechanical zero as SimpleFOC `sensor_offset`, and then saves the candidate calibration to EEPROM flash only after final confirmation. If no valid EEPROM calibration exists at boot, the firmware creates a default record from `board_config.h`.

Normal sketch upload keeps the EEPROM flash sector intact.

## Python Client

`../software/motor_usb/client.py` wraps the packet protocol and runs a background RX thread.
Scripts live directly in `software/` and import `MotorUsbController` from the
local `motor_usb` package, with no project installation or `sys.path` changes.
For example, from `firmware/`, run `python3 ../software/demo_sine_position.py`.

Client behavior:

- Runtime faults or loss of readiness on an enabled motor are detected automatically and printed to stderr. The error stays latched in the client: `update()`, `poll()` and state/echo waits raise `RuntimeError`, stopping the command loop. Create a new controller after firmware recovery to resume; an old loop cannot restart automatically. Closing still sends a disabled, zero-torque command.
- `MotorUsbController.initialize()` sends a zero-gain, zero-timeout command, waits for core1 to echo its command index, and raises an error if the requested motors are not ready or a fault is latched.
- `robot.check_ready()` checks the latest state and raises an error if either motor is unavailable or faulted; pass `flags` to check only the selected motors.
- `robot.m0.q`, `robot.m0.v`, `robot.m0.v_highfrequency`, `robot.m0.i`, and `robot.m0.i_target` are measured values updated by the background RX thread. `robot.m1` exposes the same fields.
- `robot.m0.set(...)` and `robot.m1.set(...)` update the next command targets. The target fields are `q_target`, `v_target`, `kp`, `kd`, and `iff`.
- `robot.update(block=False)` sends a command, rate-limited to 2 kHz by default. State packets continue to be parsed asynchronously.
- `robot.update(wait_state=True)` sends the command and waits for the next state packet, without requiring command-index echo.
- `robot.update(block=True)` sends the command and waits until a state packet reports the same `latest_command_index`.
- `robot.poll()` returns the latest parsed state snapshot without sending a command.
- `robot.wait_for_state()` waits for a new state packet.
- `robot.zero_torque()` sends zero gains with watchdog disabled.
