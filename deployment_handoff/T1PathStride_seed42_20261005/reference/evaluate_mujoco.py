"""Headless fixed-command MuJoCo check; writes measured speed and stability data."""

import argparse
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import torch
import yaml

from utils.model import ActorCritic, expand_appended_observations
from utils.symmetry import mirror_action, mirror_observation


def inverse_rotate(quat_xyzw, vec):
    w = quat_xyzw[3]
    xyz = quat_xyzw[:3]
    return vec * (2.0 * w * w - 1.0) - np.cross(xyz, vec) * (2.0 * w) + xyz * (np.dot(xyz, vec) * 2.0)


def yaw_from_wxyz(q):
    w, x, y, z = q
    return np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def straight_path_observation(cfg, command, xy, yaw, start_xy, start_yaw):
    if not cfg["env"].get("observe_straight_path", False):
        return np.empty(0, dtype=np.float32)
    if command[0] <= 0.2 or abs(command[1]) >= 0.05 or abs(command[2]) >= 0.05:
        return np.zeros(2, dtype=np.float32)
    delta = xy - start_xy
    lateral = -np.sin(start_yaw) * delta[0] + np.cos(start_yaw) * delta[1]
    yaw_delta = yaw - start_yaw
    heading = np.arctan2(np.sin(yaw_delta), np.cos(yaw_delta))
    return np.array([
        np.clip(lateral, -10.0, 10.0) * cfg["normalization"]["straight_lateral"],
        heading * cfg["normalization"]["straight_heading"],
    ], dtype=np.float32)


def load_actor(path, cfg):
    if path.suffix == ".pt":
        actor = torch.jit.load(str(path), map_location="cpu")
    else:
        model = ActorCritic(cfg["env"]["num_actions"], cfg["env"]["num_observations"], cfg["env"]["num_privileged_obs"])
        state = torch.load(path, map_location="cpu", weights_only=True)["model"]
        state = expand_appended_observations(model, state, cfg["basic"].get("appended_observations_from_checkpoint", 0))
        model.load_state_dict(state)
        actor = model.actor
    actor.eval()
    return actor


def run_case(cfg, actor, command, duration):
    model = mujoco.MjModel.from_xml_path(cfg["asset"]["mujoco_file"])
    model.opt.timestep = cfg["sim"]["dt"]
    data = mujoco.MjData(model)
    default = np.zeros(model.nu, dtype=np.float32)
    kp = np.zeros(model.nu, dtype=np.float32)
    kd = np.zeros(model.nu, dtype=np.float32)
    urdf = ET.parse(cfg["asset"]["file"]).getroot()
    urdf_velocity_limits = {
        joint.attrib["name"]: float(joint.find("limit").attrib["velocity"])
        for joint in urdf.findall("joint") if joint.find("limit") is not None
        and "velocity" in joint.find("limit").attrib
    }
    joint_names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, j) for j in range(model.nu)]
    if any(name not in urdf_velocity_limits for name in joint_names):
        raise ValueError("MuJoCo actuator is missing its URDF joint velocity limit")
    velocity_limits = np.asarray([urdf_velocity_limits[name] for name in joint_names])
    for j in range(model.nu):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, j)
        default[j] = next((val for key, val in cfg["init_state"]["default_joint_angles"].items() if key in name), cfg["init_state"]["default_joint_angles"]["default"])
        gain = next((key for key in cfg["control"]["stiffness"] if key in name), None)
        if gain is None:
            raise ValueError(f"Missing PD gains for {name}")
        kp[j] = cfg["control"]["stiffness"][gain]
        kd[j] = cfg["control"]["damping"][gain]
    data.qpos[:] = np.concatenate((cfg["init_state"]["pos"], [1.0, 0.0, 0.0, 0.0], default))
    mujoco.mj_forward(model, data)
    action = np.zeros(cfg["env"]["num_actions"], dtype=np.float32)
    action_names = cfg["control"].get("action_joint_names")
    action_ids = np.array([
        next(i for i in range(model.nu) if mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i) == name)
        for name in action_names
    ], dtype=int) if action_names else np.arange(model.nu)
    action_scales = np.asarray(cfg["control"].get("action_scales", [cfg["control"]["action_scale"]] * len(action)), dtype=float)
    if len(action_ids) != len(action):
        raise ValueError("Action mapping does not match policy output")
    target = default.copy()
    phase = 0.0
    rule = cfg["commands"].get("gait_frequency_rule")
    moving = any(abs(x) > 1e-8 for x in command)
    frequency = 0.0 if not moving else float(np.clip(rule["base"] + rule["forward_gain"] * max(0.0, command[0]), rule["minimum"], rule["maximum"])) if rule else float(np.mean(cfg["commands"]["gait_frequency"]))
    steps = int(duration / model.opt.timestep)
    stride = cfg["control"]["decimation"]
    samples = []
    max_torque_ratio = 0.0
    initial_yaw = yaw_from_wxyz(data.qpos[3:7])
    initial_xy = data.qpos[:2].copy()
    previous_yaw = initial_yaw
    accumulated_yaw = 0.0
    saturated_steps = 0
    saturated_joint_steps = 0
    saturated_joint_counts = np.zeros(model.nu, dtype=np.int64)
    velocity_limit_steps = 0
    velocity_near_limit_steps = 0
    velocity_limit_joint_steps = np.zeros(model.nu, dtype=np.int64)
    peak_velocity_ratios = np.zeros(model.nu)
    ground_geom = model.geom("ground").id
    foot_geoms = {
        "left": {model.geom("left_foot_1").id, model.geom("left_foot_2").id},
        "right": {model.geom("right_foot_1").id, model.geom("right_foot_2").id},
    }
    previous_foot_contact = {side: False for side in foot_geoms}
    foot_strikes = {side: [] for side in foot_geoms}
    arm_deviations = []
    waist_deviations = []
    hip_pitch_deviations = []
    arm_leg_errors = []
    phase_bins = 24
    leg_phase_sum = np.zeros((phase_bins, 2, 6), dtype=float)
    leg_phase_count = np.zeros(phase_bins, dtype=np.int64)
    policy_mirror_errors = []
    with torch.no_grad():
        for step in range(steps):
            if step % stride == 0:
                qpos = data.qpos[7:].astype(np.float32)
                qvel = data.qvel[6:].astype(np.float32)
                if model.nu == 23 and moving:
                    phase_bin = min(int(phase * phase_bins), phase_bins - 1)
                    leg_phase_sum[phase_bin, 0] += qpos[11:17] - default[11:17]
                    leg_phase_sum[phase_bin, 1] += qpos[17:23] - default[17:23]
                    leg_phase_count[phase_bin] += 1
                quat = data.sensor("orientation").data[[1, 2, 3, 0]].astype(np.float32)
                gravity = inverse_rotate(quat, np.array([0.0, 0.0, -1.0], dtype=np.float32))
                obs = np.concatenate((
                    gravity * cfg["normalization"]["gravity"],
                    data.sensor("angular-velocity").data.astype(np.float32) * cfg["normalization"]["ang_vel"],
                    np.asarray(command, dtype=np.float32) * [cfg["normalization"]["lin_vel"], cfg["normalization"]["lin_vel"], cfg["normalization"]["ang_vel"]],
                    [np.cos(2 * np.pi * phase) * moving, np.sin(2 * np.pi * phase) * moving],
                    (qpos - default) * cfg["normalization"]["dof_pos"],
                    qvel * cfg["normalization"]["dof_vel"], action,
                    straight_path_observation(
                        cfg, command, data.qpos[:2], yaw_from_wxyz(data.qpos[3:7]), initial_xy, initial_yaw
                    ),
                )).astype(np.float32)
                if len(obs) != cfg["env"]["num_observations"]:
                    raise ValueError(f"Observation length {len(obs)} does not match config")
                obs_tensor = torch.from_numpy(obs).unsqueeze(0)
                raw_action = actor(obs_tensor)
                if moving and cfg["env"]["num_actions"] in (12, 21):
                    mirrored_action = actor(mirror_observation(obs_tensor))
                    policy_mirror_errors.append(float(torch.mean((mirrored_action - mirror_action(raw_action)) ** 2).sqrt()))
                action = np.clip(raw_action.squeeze(0).numpy(), -cfg["normalization"]["clip_actions"], cfg["normalization"]["clip_actions"])
                if cfg["control"].get("zero_upper_actions", False):
                    action[:9] = 0.0
                elif cfg["control"].get("upper_action_clip") is not None:
                    action[:9] = np.clip(action[:9], -cfg["control"]["upper_action_clip"], cfg["control"]["upper_action_clip"])
                target[:] = default
                target[action_ids] += action_scales * action
                if model.nu == 23:
                    arm_deviations.append(float(np.mean(np.abs(qpos[2:10] - default[2:10]))))
                    waist_deviations.append(float(abs(qpos[10] - default[10])))
                    shoulder = qpos[[2, 6]] - default[[2, 6]]
                    hip = qpos[[11, 17]] - default[[11, 17]]
                    hip_pitch_deviations.append(float(np.mean(np.abs(hip))))
                    arm_leg_errors.append(float(np.mean(np.abs(shoulder + 0.65 * hip))))
                quat_base = data.qpos[3:7]
                body_v = inverse_rotate(np.asarray([quat_base[1], quat_base[2], quat_base[3], quat_base[0]]), data.qvel[:3])
                samples.append((data.time, float(body_v[0]), float(body_v[1]), float(data.qvel[5]), float(data.qpos[2])))
            torque = kp * (target - data.qpos[7:]) - kd * data.qvel[6:]
            limit = np.maximum(np.abs(model.actuator_ctrlrange).max(axis=1), 1e-6)
            ratio = np.abs(torque) / limit
            max_torque_ratio = max(max_torque_ratio, float(np.max(ratio)))
            saturated_steps += bool(np.any(ratio > 1.0))
            saturated_joint_steps += int(np.count_nonzero(ratio > 1.0))
            saturated_joint_counts += ratio > 1.0
            data.ctrl[:] = np.clip(torque, model.actuator_ctrlrange[:, 0], model.actuator_ctrlrange[:, 1])
            mujoco.mj_step(model, data)
            velocity_ratio = np.abs(data.qvel[6:]) / velocity_limits
            velocity_limit_joint_steps += (velocity_ratio >= 1.0)
            velocity_limit_steps += bool(np.any(velocity_ratio >= 1.0))
            velocity_near_limit_steps += bool(np.any(velocity_ratio >= 0.8))
            peak_velocity_ratios = np.maximum(peak_velocity_ratios, velocity_ratio)
            contact_geoms = [(contact.geom1, contact.geom2) for contact in data.contact]
            for side, geoms in foot_geoms.items():
                touching = any(
                    (geom1 == ground_geom and geom2 in geoms)
                    or (geom2 == ground_geom and geom1 in geoms)
                    for geom1, geom2 in contact_geoms
                )
                if touching and not previous_foot_contact[side] and data.time >= 1.0:
                    strikes = foot_strikes[side]
                    if not strikes or data.time - strikes[-1][0] >= 0.4:
                        strikes.append((float(data.time), float(data.qpos[0])))
                previous_foot_contact[side] = touching
            current_yaw = yaw_from_wxyz(data.qpos[3:7])
            accumulated_yaw += (current_yaw - previous_yaw + np.pi) % (2 * np.pi) - np.pi
            previous_yaw = current_yaw
            phase = (phase + model.opt.timestep * frequency) % 1.0
            if data.qpos[2] < cfg["rewards"]["terminate_height"] or not np.isfinite(data.qpos).all():
                break
    tail = np.asarray(samples[len(samples) // 2:], dtype=float)
    strike_periods = [current[0] - previous[0] for strikes in foot_strikes.values()
                      for previous, current in zip(strikes, strikes[1:])]
    stride_advances = [current[1] - previous[1] for strikes in foot_strikes.values()
                       for previous, current in zip(strikes, strikes[1:])]
    leg_phase_mean = leg_phase_sum / np.maximum(leg_phase_count[:, None, None], 1)
    mirrored_right = np.roll(leg_phase_mean[:, 1], phase_bins // 2, axis=0)
    mirrored_right *= np.array([1.0, -1.0, -1.0, 1.0, 1.0, -1.0])
    valid_phase = (leg_phase_count >= 2) & (np.roll(leg_phase_count, phase_bins // 2) >= 2)
    leg_symmetry_by_joint = np.sqrt(np.mean((leg_phase_mean[valid_phase, 0] - mirrored_right[valid_phase]) ** 2, axis=0)) if valid_phase.any() else None
    leg_symmetry_rms = float(np.sqrt(np.mean(leg_symmetry_by_joint ** 2))) if leg_symmetry_by_joint is not None else None
    foot_cycle_by_side = {
        side: {
            "mean_period_s": round(float(np.mean([b[0] - a[0] for a, b in zip(strikes, strikes[1:])])), 3) if len(strikes) >= 2 else None,
            "mean_stride_m": round(float(np.mean([b[1] - a[1] for a, b in zip(strikes, strikes[1:])])), 3) if len(strikes) >= 2 else None,
        }
        for side, strikes in foot_strikes.items()
    }
    return {
        "command_vx_vy_yaw": list(command), "planned_s": duration, "survived_s": round(float(data.time), 3),
        "fell": bool(data.time < duration - model.opt.timestep),
        "mean_body_vx_m_s": round(float(tail[:, 1].mean()), 3),
        "mean_body_vy_m_s": round(float(tail[:, 2].mean()), 3),
        "mean_yaw_rate_rad_s": round(float(tail[:, 3].mean()), 3),
        "lateral_drift_m": round(float(data.qpos[1]), 3),
        "final_yaw_rad": round(float(accumulated_yaw), 3),
        "minimum_base_height_m": round(float(np.asarray(samples)[:, 4].min()), 3),
        "peak_unclipped_torque_ratio": round(max_torque_ratio, 3),
        "saturation_step_fraction": round(saturated_steps / (step + 1), 3),
        "saturation_joint_step_fraction": round(saturated_joint_steps / ((step + 1) * model.nu), 3),
        "torque_request_saturation_by_joint": {
            name: round(float(count) / (step + 1), 4)
            for name, count in zip(joint_names, saturated_joint_counts)
        },
        "urdf_joint_velocity_limit_step_fraction": round(velocity_limit_steps / (step + 1), 4),
        "urdf_joint_velocity_80pct_step_fraction": round(velocity_near_limit_steps / (step + 1), 4),
        "urdf_joint_velocity_limit_joint_step_fraction": round(float(velocity_limit_joint_steps.sum()) / ((step + 1) * model.nu), 4),
        "peak_urdf_joint_velocity_ratio": round(float(peak_velocity_ratios.max()), 3),
        "urdf_joint_velocity_by_joint": {
            name: {"limit_rad_s": round(float(limit), 3),
                   "peak_ratio": round(float(peak), 3),
                   "at_limit_fraction": round(float(count) / (step + 1), 4)}
            for name, limit, peak, count in zip(joint_names, velocity_limits, peak_velocity_ratios, velocity_limit_joint_steps)
        },
        "commanded_gait_cycle_hz": round(float(frequency), 3),
        "observed_foot_cycle_hz": round(1.0 / float(np.mean(strike_periods)), 3) if strike_periods else None,
        "observed_base_advance_per_stride_m": round(float(np.mean(stride_advances)), 3) if stride_advances else None,
        "observed_foot_strikes": {side: len(strikes) for side, strikes in foot_strikes.items()},
        "observed_foot_cycle_by_side": foot_cycle_by_side,
        "mirrored_leg_phase_position_rms_rad": round(leg_symmetry_rms, 4) if leg_symmetry_rms is not None else None,
        "policy_mirror_action_rms": round(float(np.mean(policy_mirror_errors)), 4) if policy_mirror_errors else None,
        "mirrored_leg_phase_position_rms_by_joint_rad": {
            name: round(float(value), 4)
            for name, value in zip(("Hip_Pitch", "Hip_Roll", "Hip_Yaw", "Knee_Pitch", "Ankle_Pitch", "Ankle_Roll"), leg_symmetry_by_joint)
        } if leg_symmetry_by_joint is not None else None,
        "mean_arm_angle_deviation_rad": round(float(np.mean(arm_deviations)), 3) if arm_deviations else None,
        "peak_waist_angle_deviation_rad": round(float(np.max(waist_deviations)), 3) if waist_deviations else None,
        "mean_hip_pitch_deviation_rad": round(float(np.mean(hip_pitch_deviations)), 3) if hip_pitch_deviations else None,
        "mean_arm_leg_counter_swing_error_rad": round(float(np.mean(arm_leg_errors)), 3) if arm_leg_errors else None,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--task", required=True)
    parser.add_argument("--policy", required=True, type=Path)
    parser.add_argument("--seconds", type=float, default=12.0)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--only-vx", type=float, help="Evaluate one straight-line command for checkpoint comparisons")
    args = parser.parse_args()
    cfg = yaml.safe_load((Path("envs") / f"{args.task}.yaml").read_text(encoding="utf-8"))
    actor = load_actor(args.policy, cfg)
    if args.only_vx is None:
        cases = [(0.0, 0.0, 0.0), (-0.5, 0.0, 0.0), (0.0, 0.0, -0.6), (0.0, 0.0, 0.6)]
        cases += [(v, 0.0, 0.0) for v in (1.0, 1.2, 1.4, 1.6, 1.8)]
    else:
        cases = [(args.only_vx, 0.0, 0.0)]
    results = [run_case(cfg, actor, case, args.seconds) for case in cases]
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps({"task": args.task, "policy": str(args.policy), "cases": results}, indent=2), encoding="utf-8")
    print(args.output)


if __name__ == "__main__":
    main()
