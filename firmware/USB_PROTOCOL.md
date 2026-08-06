# USB Motor Control Protocol

Binary packets are sent over USB CDC. The baud rate value is ignored by USB CDC, but host tools use `115200` as the serial line coding.

All multi-byte values are little-endian. Packets are packed with no padding. The checksum is the XOR of every packet byte except the final checksum byte.

## Common Header

| Offset | Type | Field | Value |
| ---: | --- | --- | --- |
| 0 | `uint8` | `magic0` | `0xA5` |
| 1 | `uint8` | `magic1` | `0x5A` |
| 2 | `uint8` | `type` | `'C'` command or `'S'` state |
| 3 | `uint8` | `version` | `1` |
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

`timeout_ms = 0` disables or resets the watchdog. `timeout_ms > 0` enables the watchdog. If core1 does not consume a new valid command before the timeout expires, both motors are commanded to zero Iq and remain in no-torque mode until another valid command is consumed.

At firmware startup, no command has been consumed, so torque is disabled even though the motors may be initialized.

## State Packet, Board To PC

Type `'S'`, length `53` bytes.

Format string used by the Python helper: `<BBBBBHIIf8fBB`.

| Offset | Type | Field | Unit |
| ---: | --- | --- | --- |
| 0 | `uint8` | `magic0` | |
| 1 | `uint8` | `magic1` | |
| 2 | `uint8` | `type = 'S'` | |
| 3 | `uint8` | `version` | |
| 4 | `uint8` | `length = 53` | |
| 5 | `uint16` | `sequence` | board state sequence |
| 7 | `uint32` | `t_us` | us |
| 11 | `uint32` | `latest_command_index` | last command consumed by core1 |
| 15 | `float32` | `control_loop_us` | us |
| 19 | `float32` | `m0_q` | rad |
| 23 | `float32` | `m0_v` | rad/s |
| 27 | `float32` | `m0_i` | A |
| 31 | `float32` | `m0_i_target` | A |
| 35 | `float32` | `m1_q` | rad |
| 39 | `float32` | `m1_v` | rad/s |
| 43 | `float32` | `m1_i` | A |
| 47 | `float32` | `m1_i_target` | A |
| 51 | `uint8` | `flags` | bit0 M0 ready, bit1 M1 ready |
| 52 | `uint8` | `checksum` | XOR |

`latest_command_index` is copied only on core1 when the motor control loop consumes a command. The PC can measure command path latency by recording send time for a command index and waiting until a state packet echoes the same index.

## Recommended Startup Sequence

1. Open the USB CDC port.
2. Send a command with zero gains, zero feedforward, zero targets, and `timeout_ms = 0`.
3. Wait until `latest_command_index` in a state packet equals that command index.
4. Start the real command loop with a finite timeout, for example `timeout_ms = 20`.

The included `../software/demo/sine_command_demo.py` implements this sequence.

## Boot Calibration Mode

If the first non-newline byte received on USB CDC during the boot entry window is `!`, the firmware does not start the binary protocol. Core1 remains idle and core0 runs a text-based calibration wizard. The entry window is configured by `CALIBRATION_ENTRY_WAIT_MS`.

The wizard can run SimpleFOC electrical angle calibration, capture mechanical zero as SimpleFOC `sensor_offset`, and then saves the candidate calibration to EEPROM flash only after final confirmation. If no valid EEPROM calibration exists at boot, the firmware creates a default record from `board_config.h`.

Normal sketch upload keeps the EEPROM flash sector intact.

## Python Client

`../software/tools/motor_usb_client.py` wraps the packet protocol and runs a background RX thread:

- `MotorUsbController.initialize()` sends a zero-gain, zero-timeout command and waits for core1 to echo its command index.
- `robot.m0.q`, `robot.m0.v`, `robot.m0.i`, and `robot.m0.i_target` are measured values updated by the background RX thread. `robot.m1` exposes the same fields.
- `robot.m0.set(...)` and `robot.m1.set(...)` update the next command targets. The target fields are `q_target`, `v_target`, `kp`, `kd`, and `iff`.
- `robot.update(block=False)` sends a command, rate-limited to 1 kHz by default. State packets continue to be parsed asynchronously.
- `robot.update(wait_state=True)` sends the command and waits for the next state packet, without requiring command-index echo.
- `robot.update(block=True)` sends the command and waits until a state packet reports the same `latest_command_index`.
- `robot.poll()` returns the latest parsed state snapshot without sending a command.
- `robot.wait_for_state()` waits for a new state packet.
- `robot.zero_torque()` sends zero gains with watchdog disabled.
