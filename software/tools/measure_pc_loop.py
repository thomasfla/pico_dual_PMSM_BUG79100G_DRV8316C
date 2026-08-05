#!/usr/bin/env python3
import argparse
import statistics
import time

from motor_usb_client import MotorUsbController


def stats(values):
    return (
        f"avg={statistics.fmean(values):.3f} "
        f"min={min(values):.3f} "
        f"max={max(values):.3f} "
        f"p95={sorted(values)[int(0.95 * (len(values) - 1))]:.3f}"
    )


def main():
    parser = argparse.ArgumentParser(description="Measure PC-side USB control loop timing.")
    parser.add_argument("--seconds", type=float, default=5.0)
    parser.add_argument("--rate", type=float, default=1000.0)
    parser.add_argument("--timeout-ms", type=int, default=20)
    parser.add_argument("--port", default=None)
    parser.add_argument("--block", action="store_true", help="wait for echo after each command")
    parser.add_argument(
        "--max-command-rate",
        type=float,
        default=0.0,
        help="client-side command rate cap in Hz, 0 disables it for this test",
    )
    args = parser.parse_args()

    period = 1.0 / args.rate
    loop_ms = []
    rate_sleep_ms = []
    write_ms = []
    latency_ms = []
    missed = 0

    with MotorUsbController(
        port=args.port,
        timeout_ms=args.timeout_ms,
        max_command_rate_hz=args.max_command_rate,
    ) as robot:
        robot.initialize()
        q0, q1 = robot.m0.q, robot.m1.q
        start_drops = robot.state_drops
        deadline = time.perf_counter()
        end = deadline + args.seconds

        while time.perf_counter() < end:
            loop_start = time.perf_counter()
            robot.m0.set(q=q0, v=0.0, kp=0.0, kd=0.0)
            robot.m1.set(q=q1, v=0.0, kp=0.0, kd=0.0)
            robot.update(block=args.block)
            loop_end = time.perf_counter()

            loop_ms.append((loop_end - loop_start) * 1000.0)
            rate_sleep_ms.append(robot.last_rate_sleep_ms)
            write_ms.append(robot.last_write_ms)
            if robot.latency_ms is not None:
                latency_ms.append(robot.latency_ms)

            deadline += period
            sleep_s = deadline - time.perf_counter()
            if sleep_s > 0:
                time.sleep(sleep_s)
            else:
                missed += 1
                deadline = time.perf_counter()

        state_drops = robot.state_drops - start_drops
        latest_echo = robot.poll().get("latest_command_index", 0)
        last_sent = (robot.command_index - 1) & 0xFFFFFFFF

    print(f"samples={len(loop_ms)} missed_deadlines={missed}")
    print(f"state_drops={state_drops} last_sent={last_sent} latest_echo={latest_echo}")
    if loop_ms:
        print(f"pc_update_ms {stats(loop_ms)}")
    if rate_sleep_ms:
        print(f"rate_sleep_ms {stats(rate_sleep_ms)}")
    if write_ms:
        print(f"serial_write_ms {stats(write_ms)}")
    if latency_ms:
        print(f"echo_latency_ms {stats(latency_ms)}")


if __name__ == "__main__":
    main()
