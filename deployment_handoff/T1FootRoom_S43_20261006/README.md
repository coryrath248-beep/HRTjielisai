# FootRoom 1200 实机调试接口（只读／影子推理）

本目录配套 `trained_policies/T1FullBody16FH_FootRoom_S43_model1200_20261006/`。模型输入 80 维、输出 21 个关节动作。这里的脚本**只订阅状态并计算候选动作，不发布电机命令、不切换机器人模式**。既有 47→12 实机程序不受影响，可以照原流程回退。

## 到场后先采什么

在机器人计算机上装好 `booster_robotics_sdk_python`、`numpy`、`PyYAML`、`torch`。可选手柄采集还需 `evdev`、`sshkeyboard`。在仓库根目录执行：

```bash
python deployment_handoff/T1FootRoom_S43_20261006/probe.py \
  --net 127.0.0.1 --seconds 60 --output /tmp/t1_probe.jsonl
```

脚本记录 23 路串联电机和并联电机的 `q/dq/tau_est`、IMU 与里程计 `x/y/theta`，每条都带单调时间戳。可加 `--remote` 同时记录原手柄的 `vx/vy/yaw`，此时 `--max-vx` 默认 1.6 m/s，`--max-yaw` 默认 0.5 rad/s。采集是只读的；仍须由现场人员确保机器人处于合适状态。

先检查输出末尾的计数：`state` 必须大于零，`bad_count/callback_error` 必须为零；`odom` 若为零，说明当前 SDK 或机器未提供可用里程计，80 维策略的直线误差项尚不能实机复现。采集文件可能包含设备状态，不要直接公开上传。

`python deployment_handoff/T1FootRoom_S43_20261006/inspect_capture.py /tmp/t1_probe.jsonl` 会列出采样间隔、里程计更新、电机数量及各串联索引观测到的速度和估计力矩峰值。这些峰值是采样结果，不能直接当作额定限值。

## 影子推理与接口核对

复制 `profile.example.json` 为本机专用 profile，填机器人标识并逐项核对：23 路电机顺序、头臂腰腿符号、串联 crank 与训练踝俯仰／横滚的对应关系、IMU 坐标系、里程计坐标系和方向。示例 profile 的 23 路默认取 `serial`，**仅用于观察与排错，不能据此确认踝部映射**。如需改用并联状态，在 `joint_state_mapping` 中把相应项改成 `parallel:<索引>`，现场对照 SDK 数据验证后才把对应 `*_verified` 设为 `true`。

采了手柄命令的日志：

```bash
python deployment_handoff/T1FootRoom_S43_20261006/replay.py /tmp/t1_probe.jsonl \
  --profile /tmp/t1_profile.json --output /tmp/t1_shadow.jsonl
```

未采手柄命令时，可以用 `--command 0.8 0 0` 做离线检查；前进加转向可用 `--command 0.8 0 0.2`。输出逐帧保存 80 维观察、21 动作、23 路训练坐标系目标角、横向误差、航向误差和状态。若缺里程计、超过 100 ms 未更新、输入维度不符或时间跳变，记录 `error`，不会用零值伪装正常状态。输出中的 `contract_verified` 只有四项现场核对都完成才为真；即使为真，也只是接口核对，不代表可以直接下发电机目标。

## 手动微调漂移

可以在 `vx>0` 时用摇杆给少量 `yaw` 修正。调试逻辑把绝对值小于 **0.06 rad/s** 的转向命令视为手柄零位噪声；超过该值时按“人工转向”处理，并把直线偏差项暂置零。松开转向、回到 `yaw=0` 后，以当时位置和朝向开启新直线段，避免策略试图拉回转向前的旧路线。这个动作会改变行进方向，**不能替代长期航向／横漂反馈**。先在低速记录左右相同指令的实际偏航速率，再决定操作者需要的微调量；永久固定偏置要查零点、执行器和传感器，不应只靠摇杆掩盖。

原手柄服务本身还有绝对值 0.1 的输入阈值，因此从 `probe.py --remote` 读到的实际微调起点通常是 0.1 rad/s；影子推理的 0.06 阈值负责防止更小的输入误触直行／转向切换。现场要核对摇杆正负方向，不能由屏幕上的命令符号推断机器人会往哪侧转。

## 实机动作前仍需核对

SDK 的踝部串联索引是 `CrankUp/CrankDown`，训练 URDF 是 `Ankle_Pitch/Ankle_Roll`。必须确认状态换算及命令换算，不能把影子推理的训练关节目标直接送给串联电机。还需确认机器人确实为 23 自由度版本、上肢／腰低层控制可用、PD 增益和力矩限值、控制频率与延迟、急停及回退流程。所有这些依赖现场机器；本目录保留了明确的核对位置，不预填“已验证”。

`python -m unittest discover -s deployment_handoff/T1FootRoom_S43_20261006 -p 'test_*.py'` 可在有 PyTorch/Numpy/PyYAML 的开发机运行接口与转向切换测试。
