"""Read-only Booster T1 telemetry and optional joystick-command capture.

This script does not import a low-command publisher or change robot mode.
"""

import argparse
import json
from pathlib import Path
import sys
import threading
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--net", default="127.0.0.1", help="SDK network interface")
    parser.add_argument("--seconds", type=float, default=30.0)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--sample-hz", type=float, default=50.0)
    parser.add_argument("--remote", action="store_true", help="log the existing joystick/keyboard commands")
    parser.add_argument("--max-vx", type=float, default=1.6)
    parser.add_argument("--max-vy", type=float, default=0.5)
    parser.add_argument("--max-yaw", type=float, default=0.5)
    args = parser.parse_args()
    if args.seconds <= 0 or args.sample_hz <= 0:
        parser.error("seconds and sample-hz must be positive")

    from booster_robotics_sdk_python import ChannelFactory, B1JointCnt, B1LowStateSubscriber
    try:
        from booster_robotics_sdk_python import B1OdometerStateSubscriber
    except ImportError:
        B1OdometerStateSubscriber = None

    remote = None
    if args.remote:
        deploy = Path(__file__).resolve().parents[2] / "train_kit" / "deploy"
        sys.path.insert(0, str(deploy))
        from utils.remote_control_service import JoystickConfig, RemoteControlService
        remote = RemoteControlService(JoystickConfig(max_vx=args.max_vx, max_vy=args.max_vy,
                                                      max_vyaw=args.max_yaw))

    args.output.parent.mkdir(parents=True, exist_ok=True)
    lock = threading.Lock()
    counters = {"state": 0, "odom": 0, "bad_count": 0, "callback_error": 0}
    last_sample = [0.0]
    with args.output.open("w", encoding="utf-8", buffering=1) as log:
        def write(record):
            with lock:
                log.write(json.dumps(record, ensure_ascii=False, separators=(",", ":")) + "\n")

        def on_state(message):
            try:
                now = time.monotonic()
                if now - last_sample[0] < 1.0 / args.sample_hz:
                    return
                last_sample[0] = now
                serial = message.motor_state_serial
                parallel = getattr(message, "motor_state_parallel", [])
                if len(serial) != B1JointCnt:
                    counters["bad_count"] += 1
                record = {
                    "kind": "state", "monotonic_s": now, "wall_ns": time.time_ns(),
                    "motor_count": len(serial), "parallel_count": len(parallel),
                    "imu_rpy": list(message.imu_state.rpy),
                    "imu_gyro": list(message.imu_state.gyro),
                    "imu_acc": list(message.imu_state.acc),
                    "serial": [{"q": m.q, "dq": m.dq, "tau_est": getattr(m, "tau_est", None)} for m in serial],
                    "parallel": [{"q": m.q, "dq": m.dq, "tau_est": getattr(m, "tau_est", None)} for m in parallel],
                }
                if remote is not None:
                    record["command"] = [remote.get_vx_cmd(), remote.get_vy_cmd(), remote.get_vyaw_cmd()]
                write(record)
                counters["state"] += 1
            except Exception as exc:
                counters["callback_error"] += 1
                print("low-state callback error: %s" % exc, file=sys.stderr)

        def on_odom(message):
            try:
                record = {"kind": "odom", "monotonic_s": time.monotonic(), "wall_ns": time.time_ns(),
                          "x": message.x, "y": message.y, "theta": message.theta}
                write(record)
                counters["odom"] += 1
            except Exception as exc:
                counters["callback_error"] += 1
                print("odometer callback error: %s" % exc, file=sys.stderr)

        ChannelFactory.Instance().Init(0, args.net)
        state_sub = B1LowStateSubscriber(on_state)
        state_sub.InitChannel()
        odom_sub = None
        if B1OdometerStateSubscriber is not None:
            odom_sub = B1OdometerStateSubscriber(on_odom)
            odom_sub.InitChannel()
        print("READ ONLY: capturing %s for %.1f seconds" % (args.output, args.seconds))
        try:
            time.sleep(args.seconds)
        except KeyboardInterrupt:
            pass
        finally:
            if remote is not None:
                remote.close()
            print(json.dumps(counters, ensure_ascii=False))
            if not counters["state"] or counters["bad_count"] or counters["callback_error"]:
                raise RuntimeError("telemetry capture contains invalid states; inspect log and SDK")


if __name__ == "__main__":
    main()
