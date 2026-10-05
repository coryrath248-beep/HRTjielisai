"""FootRoom 1200 policy contract and read-only shadow inference.

No SDK publisher is imported here. The adapter produces diagnostic targets only;
the robot's joint/ankle calibration must be checked before any actuator uses them.
"""

from dataclasses import dataclass
from pathlib import Path
import hashlib
import io
import math

import numpy as np
import torch
import yaml


ROOT = Path(__file__).resolve().parents[2]
MILESTONE = ROOT / "trained_policies" / "T1FullBody16FH_FootRoom_S43_model1200_20261006"
CONFIG = MILESTONE / "T1FullBody16FH_FootRoom_S43.yaml"
MODEL = MILESTONE / "model_1200.pt"
MODEL_SHA256 = "d6756419268ba789abf8c2fcbedd2bb48e1eef1f66f36d7ecd466c4258f2d05e"

# Matches the 23-DoF T1 SDK JointIndex order. SDK ankle entries refer to cranks;
# their relationship to URDF ankle pitch/roll must be checked on each robot.
JOINT_NAMES = (
    "AAHead_yaw", "Head_pitch",
    "Left_Shoulder_Pitch", "Left_Shoulder_Roll", "Left_Elbow_Pitch", "Left_Elbow_Yaw",
    "Right_Shoulder_Pitch", "Right_Shoulder_Roll", "Right_Elbow_Pitch", "Right_Elbow_Yaw",
    "Waist",
    "Left_Hip_Pitch", "Left_Hip_Roll", "Left_Hip_Yaw", "Left_Knee_Pitch",
    "Left_Ankle_Pitch", "Left_Ankle_Roll",
    "Right_Hip_Pitch", "Right_Hip_Roll", "Right_Hip_Yaw", "Right_Knee_Pitch",
    "Right_Ankle_Pitch", "Right_Ankle_Roll",
)
ANKLE_INDICES = (15, 16, 21, 22)


def _array(values, size, name):
    result = np.asarray(values, dtype=np.float32)
    if result.shape != (size,) or not np.all(np.isfinite(result)):
        raise ValueError("%s must contain %d finite values" % (name, size))
    return result


def gravity_from_rpy(rpy):
    """R(q)^T [0,0,-1], in the SDK's roll/pitch/yaw convention."""
    roll, pitch, _ = _array(rpy, 3, "rpy")
    return np.array((math.sin(pitch), -math.sin(roll) * math.cos(pitch),
                     -math.cos(roll) * math.cos(pitch)), dtype=np.float32)


def wrap_angle(value):
    return math.atan2(math.sin(value), math.cos(value))


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    theta: float

    def checked(self):
        if not all(math.isfinite(v) for v in (self.x, self.y, self.theta)):
            raise ValueError("odometry pose is not finite")
        return self


class ManualPathReference:
    """Reset the straight-line origin after a manual steering or stop segment."""

    def __init__(self, yaw_deadband=0.06):
        if yaw_deadband <= 0.05:
            raise ValueError("yaw deadband must exceed the policy straight-mode threshold 0.05")
        self.yaw_deadband = float(yaw_deadband)
        self.origin = None
        self.was_straight = False

    def update(self, command, pose):
        vx, vy, yaw = _array(command, 3, "command")
        # A small stick bias stays at zero. An intentional nudge must cross
        # 0.05 rad/s so the policy's straight-path feedback does not fight it.
        if abs(yaw) < self.yaw_deadband:
            yaw = np.float32(0.0)
        effective = np.array((vx, vy, yaw), dtype=np.float32)
        straight = vx > 0.2 and abs(vy) < 0.05 and abs(yaw) < 0.05
        if not straight:
            self.was_straight = False
            return effective, (0.0, 0.0), "manual_turn" if abs(yaw) >= 0.05 else "other"
        if pose is None:
            raise ValueError("fresh odometry is required for straight-path observations")
        pose = pose.checked()
        if not self.was_straight:
            self.origin = pose
        self.was_straight = True
        dx, dy = pose.x - self.origin.x, pose.y - self.origin.y
        lateral = -math.sin(self.origin.theta) * dx + math.cos(self.origin.theta) * dy
        heading = wrap_angle(pose.theta - self.origin.theta)
        return effective, (lateral, heading), "straight"


class FootRoomShadow:
    """Construct the exact actor layout and infer targets without publishing."""

    def __init__(self, config_path=CONFIG, model_path=MODEL, yaw_deadband=0.06):
        with open(config_path, encoding="utf-8") as stream:
            cfg = yaml.safe_load(stream)
        if cfg["env"]["num_observations"] != 80 or cfg["env"]["num_actions"] != 21:
            raise ValueError("expected FootRoom 80-observation / 21-action config")
        self.cfg = cfg
        self.policy_dt = float(cfg["sim"]["dt"] * cfg["control"]["decimation"])
        self.action_names = tuple(cfg["control"]["action_joint_names"])
        if len(self.action_names) != 21 or not set(self.action_names).issubset(JOINT_NAMES):
            raise ValueError("unexpected action joint mapping")
        self.action_indices = np.array([JOINT_NAMES.index(name) for name in self.action_names])
        self.action_scales = _array(cfg["control"]["action_scales"], 21, "action scales")
        defaults = cfg["init_state"]["default_joint_angles"]
        self.default_q = np.array([
            defaults.get(name, defaults.get("Hip_Pitch" if "Hip_Pitch" in name else
                                    "Knee_Pitch" if "Knee_Pitch" in name else
                                    "Ankle_Pitch" if "Ankle_Pitch" in name else "default"))
            for name in JOINT_NAMES
        ], dtype=np.float32)
        if not np.all(np.isfinite(self.default_q)):
            raise ValueError("missing default joint angle")
        # Torch's Windows C++ loader may reject non-ASCII filesystem paths.
        with open(model_path, "rb") as stream:
            model_bytes = stream.read()
        if hashlib.sha256(model_bytes).hexdigest() != MODEL_SHA256:
            raise ValueError("FootRoom actor SHA256 mismatch")
        self.model = torch.jit.load(io.BytesIO(model_bytes), map_location="cpu").eval()
        self.last_action = np.zeros(21, dtype=np.float32)
        self.gait_phase = 0.0
        self.last_time = None
        self.path = ManualPathReference(yaw_deadband)

    def _frequency(self, command):
        if np.max(np.abs(command)) <= 1.0e-8:
            return 0.0
        rule = self.cfg["commands"]["gait_frequency_rule"]
        return float(np.clip(rule["base"] + rule["forward_gain"] * max(float(command[0]), 0.0),
                             rule["minimum"], rule["maximum"]))

    def step(self, stamp, q, dq, rpy, gyro, command, pose):
        if not math.isfinite(stamp) or (self.last_time is not None and stamp <= self.last_time):
            raise ValueError("policy timestamps must increase")
        q = _array(q, 23, "joint position")
        dq = _array(dq, 23, "joint velocity")
        gyro = _array(gyro, 3, "IMU angular velocity")
        effective, (lateral, heading), mode = self.path.update(command, pose)
        if self.last_time is not None:
            elapsed = stamp - self.last_time
            if elapsed > 0.1:
                raise ValueError("state gap exceeds 100 ms; reset shadow before continuing")
            self.gait_phase = (self.gait_phase + elapsed * self._frequency(effective)) % 1.0
        self.last_time = stamp
        frequency = self._frequency(effective)
        norm = self.cfg["normalization"]
        active = float(frequency > 0.0)
        obs = np.concatenate((
            gravity_from_rpy(rpy) * norm["gravity"],
            gyro * norm["ang_vel"],
            effective * np.array((norm["lin_vel"], norm["lin_vel"], norm["ang_vel"]), dtype=np.float32),
            np.array((math.cos(2 * math.pi * self.gait_phase) * active,
                      math.sin(2 * math.pi * self.gait_phase) * active), dtype=np.float32),
            (q - self.default_q) * norm["dof_pos"],
            dq * norm["dof_vel"],
            self.last_action,
            np.array((np.clip(lateral, -10.0, 10.0) * norm["straight_lateral"],
                      heading * norm["straight_heading"]), dtype=np.float32),
        )).astype(np.float32)
        if obs.shape != (80,) or not np.all(np.isfinite(obs)):
            raise ValueError("invalid 80-dimensional observation")
        with torch.inference_mode():
            raw = self.model(torch.from_numpy(obs).unsqueeze(0)).cpu().numpy().reshape(-1)
        action = np.clip(_array(raw, 21, "model action"), -norm["clip_actions"], norm["clip_actions"])
        self.last_action[:] = action
        targets = self.default_q.copy()
        targets[self.action_indices] += self.action_scales * action
        return {"mode": mode, "command": effective.tolist(), "lateral_m": lateral,
                "heading_rad": heading, "observation": obs.tolist(),
                "action": action.tolist(), "targets_urdf_rad": targets.tolist()}
