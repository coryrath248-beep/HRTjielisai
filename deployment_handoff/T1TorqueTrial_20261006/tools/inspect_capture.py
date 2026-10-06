"""Summarize read-only telemetry before attempting 80-dimensional shadow replay."""

import argparse
import json
import math
from pathlib import Path


def gaps(stamps):
    if len(stamps) < 2:
        return {"count": len(stamps), "median_dt_ms": None, "max_dt_ms": None}
    values = sorted((b - a) * 1000 for a, b in zip(stamps, stamps[1:]))
    return {"count": len(stamps), "median_dt_ms": round(values[len(values) // 2], 2),
            "max_dt_ms": round(values[-1], 2)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture", type=Path)
    args = parser.parse_args()
    state_times, odom_times = [], []
    serial_counts, parallel_counts = set(), set()
    peak_dq, peak_tau, peak_temp = [0.0] * 23, [None] * 23, [None] * 23
    odom_first = odom_last = None
    command_seen = False
    with args.capture.open(encoding="utf-8") as stream:
        for line in stream:
            item = json.loads(line)
            if item["kind"] == "odom":
                odom_times.append(item["monotonic_s"])
                latest_pose = (item["x"], item["y"], item["theta"])
                if odom_first is None:
                    odom_first = latest_pose
                odom_last = latest_pose
            elif item["kind"] == "state":
                state_times.append(item["monotonic_s"])
                serial_counts.add(item["motor_count"])
                parallel_counts.add(item["parallel_count"])
                command_seen |= "command" in item
                for index, motor in enumerate(item["serial"][:23]):
                    peak_dq[index] = max(peak_dq[index], abs(motor["dq"]))
                    tau = motor.get("tau_est")
                    if tau is not None and math.isfinite(tau):
                        peak_tau[index] = max(peak_tau[index] or 0.0, abs(tau))
                    temperature = motor.get("temperature")
                    if temperature is not None and math.isfinite(temperature):
                        peak_temp[index] = max(peak_temp[index] or 0.0, temperature)
    report = {
        "state": gaps(state_times), "odom": gaps(odom_times),
        "serial_motor_counts": sorted(serial_counts), "parallel_motor_counts": sorted(parallel_counts),
        "command_recorded": command_seen, "odom_first": odom_first, "odom_last": odom_last,
        "peak_abs_serial_dq_by_sdk_index_rad_s": [round(v, 3) for v in peak_dq],
        "peak_abs_serial_tau_est_by_sdk_index": [round(v, 3) if v is not None else None for v in peak_tau],
        "peak_serial_temperature_by_sdk_index": peak_temp,
    }
    print(json.dumps(report, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
