# Python scripts

Run the scripts directly from the checkout. The project does not need to be
installed, and the scripts do not modify `sys.path`.

From the repository root:

```sh
python3 software/demo_sine_position.py
```

From `firmware/`:

```sh
python3 ../software/demo_sine_position.py
```

All demos and tools are in this directory. The shared library is a local package:

```text
software/
    motor_usb/
        __init__.py
        client.py
        protocol.py
    demo_sine_position.py
    demo_motor_coupling.py
    demo_motor_tone.py
    monitor_zero_torque.py
    plot_current_step_response.py
    measure_host_timing.py
```

| Script | Purpose |
| --- | --- |
| `demo_sine_position.py` | Move both motors through a sinusoidal position trajectory. |
| `demo_motor_coupling.py` | Couple the two motors so each follows the other's motion. |
| `demo_motor_tone.py` | Generate an audible tone with alternating motor current. |
| `monitor_zero_torque.py` | Display motor state while continuously commanding zero torque. |
| `plot_current_step_response.py` | Apply a square-wave current target and plot the response. |
| `measure_host_timing.py` | Measure the Python command loop and USB echo latency. |

Place new controller scripts here and import the client with:

```python
from motor_usb import MotorUsbController
```

The client checks readiness and faults automatically in `initialize()` and
`update()`. A fault prints its cause and raises an error to stop the command loop.
See [the USB protocol](../firmware/USB_PROTOCOL.md) for the API and packet format.

Opening a serial connection uses the existing `pyserial` dependency. Plotting
also uses `matplotlib`.
