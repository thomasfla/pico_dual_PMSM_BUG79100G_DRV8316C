# Dual PMSM firmware

RP2350 / Pico 2, two DRV8316C drivers, AS5048A encoders and four parallel BU79100 ADC channels.

## Build

```sh
pio run -e pico
python3 test/run_host_tests.py
```

PlatformIO fetches upstream SimpleFOC at `f5ac6522995446ee7365e61cd0e4964dea36a5da`
(2.3.5) and the Raspberry Pi platform at `cc24cfef37ed22ca9f2a6aead28c2deb76c39f24`.
The platform pins the Arduino-Pico core and selects release toolchain archives.
No Arduino sketchbook library, local SimpleFOC patch, or private SimpleFOC PWM
parameter layout is required. The generated image is `.pio/build/pico/firmware.uf2`.

## Layout

- `main.cpp`: boot selection and the two core entry points.
- `motor_control.cpp`: hardware startup, FOC scheduling, PD references and fault handling.
- `board_pwm.cpp`: board-owned PWM slices, staged compare writes and voltage scaling.
- `checked_encoder.h`: checked SPI reads and a constant-cost velocity estimator.
- `emergency_stop.cpp`, `emergency_button.h`: polled GPIO 28 stop and deliberate reset gesture.
- `BU79100QuadReader.cpp`, `current_feedback.cpp`: PIO/DMA transport, frame identity,
  shared current samples and offset calibration.
- `usb_interface.cpp`: binary packets and core mailboxes.
- `calibration_mode.cpp`, `board_test_mode.cpp`, `motor_identification.cpp`: boot tools.

## Timing and startup

GPIOs 0–5 use PWM slices 0–2. GPIO 27 supplies the ADC trigger on slice 5.
Only these slices are initialized. At 150 MHz, phase-correct TOP=3750 gives a
50 us period. The PIO dummy frame and acquisition guard place the real sample
near the peak. The DMA interrupt reloads one 64-word block every 32 frames
(625 interrupts/s); a monotonic frame number detects new samples and skipped frames.

The control core polls for each completed frame, decodes all four channels once,
and commits the previous frame's staged outputs. It then samples both encoders
and computes the next outputs with a full 50 us period available. Sample-to-output
delay is fixed at approximately 75 us: one PWM period plus the remaining half
period until the latch. Three packed compare writes commit all six phases with
interrupts briefly disabled and at least 3 us remaining before zero. Late
computations stop the drivers. USB runs on core0. The phase check accepts delayed
frame observation during the descending half-cycle and rechecks the phase at
the compare writes.

Normal boot requires electrical calibration and never aligns automatically.
Enter `!` during boot for calibration or `?` for board tests. See
[USB_PROTOCOL.md](USB_PROTOCOL.md) for position conventions, watchdog semantics,
ready/fault bits and the command format. The packet sizes/version remain compatible.

GPIO 28 uses an internal pull-up and stops both motors when polled low. No GPIO
interrupt is used. Release after stopping, then hold the button again: recovery
starts as soon as 3 seconds elapse, even while it remains pressed. Feedback is
checked before the motors become ready, and a new USB command is required for
torque. The original stop press cannot also reset the stop; after recovery,
release and press again to trigger another stop. GPIO 28 remains an input; future shared
RGB LED use can be coordinated in the central polling function.

Inductance measurement stages the test voltage after a known ADC frame and
uses the subsequent PWM latch and sample phases to calculate elapsed time.
The first sample at or after the requested rise time is used. Driver propagation,
analog settling and PIO input synchronization introduce residual measurement
error; the printed inductance remains an estimate. The 42-clock real-CS offset
in `board_pwm.cpp` must be updated if the PIO dummy/acquisition sequence changes.

## Validation

Host tests cover ADC channel decoding, rejection of missing/railed/stale samples,
offset calibration, sharing one frame between motors, timer wrap, mechanical
zero across reboot and encoder wrap, velocity observation windows, and the
inductance timing calculation, PWM latch boundaries, polled emergency-stop
latching, reset timing and bounce, and host readiness/fault reporting. C++ tests
compile with undefined-behavior sanitization; the Python
client tests use the same `pyserial` dependency as the demos.

The connected board was verified at a 50 us control period with both motors
ready and zero Iq targets; the failing half-period scheduler had measured a
36 us control pass. A physical GPIO 28 button test produced ready `0x03`,
E-stop `0x4c`, and ready `0x03` with the previous reset-on-release gesture, with zero current
targets throughout and a 50 us control period after recovery.
Encoder CS setup/hold, the physical current acquisition
window, completion before the next ADC frame, and the six-phase PWM latch still
need oscilloscope verification. Verify fault handling with encoder disconnect,
ADC interruption and driver fault tests on the board; these faults require a
reset before normal operation can resume. Separately verify that command
timeout sets both Iq targets to zero and that a fresh command restores control.
