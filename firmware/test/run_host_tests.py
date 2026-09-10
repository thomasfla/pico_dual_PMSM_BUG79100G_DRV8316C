#!/usr/bin/env python3
"""Run deterministic control regressions without a board or PlatformIO."""
import pathlib
import subprocess
import tempfile
import sys

root = pathlib.Path(__file__).resolve().parents[1]
with tempfile.TemporaryDirectory(prefix="pmsm-tests-") as tmp:
    for name, sources in {
        "control-math": ["test/control_math_test.cpp"],
        "current-feedback": ["test/current_feedback_test.cpp", "src/current_feedback.cpp"],
        "emergency-stop": ["test/emergency_stop_test.cpp", "src/emergency_stop.cpp"],
    }.items():
        binary = pathlib.Path(tmp) / name
        subprocess.run([
            "g++", "-std=c++17", "-O2", "-Wall", "-Wextra", "-Werror",
            "-fsanitize=undefined", "-fno-sanitize-recover=all",
            "-I", str(root / "test/stubs"), "-I", str(root / "src"),
            "-I", str(root / "include"),
            *[str(root / source) for source in sources], "-o", str(binary),
        ], check=True)
        subprocess.run([str(binary)], check=True)
subprocess.run([
    sys.executable, "-B", "-m", "unittest", "discover",
    "-s", str(root / "test"), "-p", "usb_ready_test.py",
], cwd=root.parent / "software", check=True)
