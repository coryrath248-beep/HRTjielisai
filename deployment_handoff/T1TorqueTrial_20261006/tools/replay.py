"""Run a T1 actor in shadow mode over a read-only robot capture."""

import argparse
import json
from pathlib import Path

from footroom_debug import FootRoomShadow, Pose2D


def mapped_state(record, mapping):
    q, dq = [], []
    for entry in mapping:
        source, index_text = entry.split(":", 1)
        if source not in ("serial", "parallel"):
            raise ValueError("mapping source must be serial or parallel")
        motor = record[source][int(index_text)]
        q.append(motor["q"])
        dq.append(motor["dq"])
    return q, dq


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture", type=Path)
    parser.add_argument("--profile", type=Path, default=Path(__file__).with_name("profile.example.json"))
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--command", type=float, nargs=3, metavar=("VX", "VY", "YAW"),
                        help="fixed command; otherwise use commands recorded by probe --remote")
    parser.add_argument("--max-odom-age", type=float, default=0.1)
    parser.add_argument("--config", type=Path, help="policy training config; defaults to TorqueMild200")
    parser.add_argument("--model", type=Path, help="TorchScript actor; defaults to TorqueMild200")
    parser.add_argument("--sha256", help="required SHA256 when --model is supplied")
    args = parser.parse_args()
    if bool(args.model) != bool(args.sha256):
        parser.error("--model and --sha256 must be supplied together")
    with args.profile.open(encoding="utf-8") as stream:
        profile = json.load(stream)
    mapping = profile["joint_state_mapping"]
    if len(mapping) != 23:
        parser.error("joint_state_mapping must contain 23 entries")
    shadow_kwargs = {"yaw_deadband": profile["yaw_deadband_rad_s"]}
    if args.config:
        shadow_kwargs["config_path"] = args.config
    if args.model:
        shadow_kwargs["model_path"] = args.model
        shadow_kwargs["expected_model_sha256"] = args.sha256
    shadow = FootRoomShadow(**shadow_kwargs)
    verified = all(profile.get(key) is True for key in
                   ("joint_order_verified", "ankle_mapping_verified", "imu_frame_verified", "odometry_frame_verified"))
    latest_pose, pose_time = None, None
    counts = {"state": 0, "shadow": 0, "missing_command": 0, "stale_odom": 0, "rejected": 0}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.capture.open(encoding="utf-8") as source, args.output.open("w", encoding="utf-8") as output:
        for line in source:
            record = json.loads(line)
            now = record["monotonic_s"]
            if record["kind"] == "odom":
                latest_pose = Pose2D(record["x"], record["y"], record["theta"])
                pose_time = now
                continue
            if record["kind"] != "state":
                continue
            counts["state"] += 1
            command = args.command if args.command is not None else record.get("command")
            if command is None:
                counts["missing_command"] += 1
                continue
            pose = latest_pose if pose_time is not None and 0 <= now - pose_time <= args.max_odom_age else None
            if pose is None:
                counts["stale_odom"] += 1
            try:
                q, dq = mapped_state(record, mapping)
                result = shadow.step(now, q, dq, record["imu_rpy"], record["imu_gyro"], command, pose)
                result.update({"monotonic_s": now, "contract_verified": verified})
                output.write(json.dumps(result, separators=(",", ":")) + "\n")
                counts["shadow"] += 1
            except (ValueError, IndexError, KeyError) as exc:
                counts["rejected"] += 1
                output.write(json.dumps({"monotonic_s": now, "error": str(exc)}) + "\n")
                # Reset timing after dropped/stale samples. Do not silently
                # feed stale pose or fill missing observations with zeros.
                shadow.last_time = None
                shadow.path.was_straight = False
    print(json.dumps({"counts": counts, "contract_verified": verified,
                      "output": str(args.output)}, ensure_ascii=False))


if __name__ == "__main__":
    main()
