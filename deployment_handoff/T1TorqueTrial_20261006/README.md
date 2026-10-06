# T1 全身策略现场试验包（2026-10-06）

**首测候选：TorqueMild S43 第 200 轮。** 其固定直行力矩请求触限最低，已导出 80 维观察、21 动作的 TorchScript `models/mild200/model_200.pt`。`TorqueFirm S43 第 800 轮`作为速度较高的备选。两版都是仿真权重，未在实机执行过 21 动作控制；本包的 `tools/` 只能采集状态和做影子推理，**不会发布电机指令**。旧 47→12 实机程序在仓库 `train_kit/deploy/`，本包不覆盖它。

## 同条件数据

1.6 m/s 前进命令、MuJoCo 固定直行 60 秒；横漂为全程末端位移。力矩触限是仿真中**请求力矩在限幅前超出模型上限的物理步占比**，不是实机电机触限率。

| 策略 | 走满 | 实速 m/s | 横漂 m | 力矩请求触限 | 关节速度触限 | 左脚支撑朝向 rad | 躯干平均横滚 rad |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| TorqueMild 200（首测） | 60 s | 1.485 | −0.019 | 5.4% | 0% | +0.0808 | +0.0305 |
| TorqueFirm 800（备选） | 60 s | 1.518 | +0.010 | 9.4% | 0% | +0.0803 | +0.0337 |
| FootRoom 1200（原里程碑） | 60 s | 1.521 | +0.067 | 10.6% | 0% | +0.158 | +0.0475 |

Isaac Gym 默认随机地形、0.8 秒前速渐增、每颗种子 64 环境×30 秒：Mild 200 在种子 42/44 存活 63/64、62/64，后半程实速 1.492/1.493 m/s、力矩请求触限 16.0%/16.9%；Firm 800 存活 61/64、62/64，实速 1.551/1.548 m/s、触限 16.6%/17.3%。FootRoom 1200 同条件存活 62/64、61/64，实速 1.540/1.550 m/s、触限 21.2%/21.5%。Mild 的 2/128 存活差距不足以证明其普遍更稳。`evidence/` 保留原始 JSON，可复核口径。

`videos/` 为两版各 15 秒的 MuJoCo 正面、侧面 H.264 视频。15 秒平均速度受起步影响，不能与上表 60 秒指标混用。视频是仿真，不是实机。

## 包内容与校验

- `models/mild200/`、`models/firm800/`：TorchScript 权重和完整训练 YAML。训练检查点 `.pth` 留在项目本地与训练服务器，实机推理只需 `.pt`。
- `evidence/`：60 秒固定直行和双种子随机环境的原始评估。
- `tools/probe.py`：只读采集 23 路电机、IMU、里程计；可选记录手柄。
- `tools/inspect_capture.py`：检查采样周期、状态数量、峰值。
- `tools/replay.py`、`tools/footroom_debug.py`：将真实采集回放进 80→21 策略，输出观察、动作和训练坐标系的 23 路目标角；只写日志。
- `tools/profile.example.json`：现场关节、IMU、里程计核对记录。默认核对项全部为 `false`。

先运行 `python verify_package.py`，确认两个 TorchScript 哈希、YAML 维度和一次 80→21 推理。包内 `SHA256SUMS.txt` 记录各文件摘要。**Mild .pt** 的 SHA256 为 `f3fe8ddd3d66e4441b4840448bd0eec6954cd7749054a204966f6895aa9bba57`；**Firm .pt** 为 `5e8786e5fff1506345193570908d340df5c8cb864b7018cf268f033ea18bd8a8`。

## 现场顺序

1. 保留原程序和权重；先记录机器人型号、固件、SDK 版本、关节数、急停和回退操作。安装 `booster_robotics_sdk_python`、`numpy`、`PyYAML`、`torch`；手柄采集需原程序的 `evdev`、`sshkeyboard`。先运行旧程序确认原路可用。
2. 在机器人计算机只读采集：`python tools/probe.py --net 127.0.0.1 --seconds 60 --output /tmp/t1_probe.jsonl`。若要记录手柄，加 `--remote`。用 `python tools/inspect_capture.py /tmp/t1_probe.jsonl` 检查状态与里程计更新。
3. 从 `tools/profile.example.json` 复制现场 profile，核对 23 路顺序和符号、IMU 方向、里程计方向，尤其核对 SDK 的 `CrankUp/CrankDown` 与训练模型 `Ankle_Pitch/Roll` 的状态和命令换算。`serial:15/16/21/22` 只是初始采集映射，不能直接视为已验证的训练坐标系。
4. 运行首测候选影子推理：`python tools/replay.py /tmp/t1_probe.jsonl --profile /tmp/t1_profile.json --command 0.5 0 0 --output /tmp/t1_mild_shadow.jsonl`。Firm 备选加 `--config models/firm800/train.yaml --model models/firm800/model_800.pt --sha256 5e8786e5fff1506345193570908d340df5c8cb864b7018cf268f033ea18bd8a8`。检查 80 维观察、21 动作、23 路目标、时间戳和 `error` 字段。
5. 上机动作前仍需把旧 `train_kit/deploy/deploy.py` 的 SDK 发布框架接到新策略，加入里程计、0.8 秒起步渐增和完整 21 路动作映射，并在目标机器低速、受扶持条件下核对踝部换算、上下肢/腰、PD 增益和限幅。当前仓库还没有通过该现场核对的 21 动作电机发布程序，**不能把 `.pt` 替换旧 12 动作权重后直接行走**。

手动转向时可在前进命令上叠加小幅 `yaw`，影子逻辑以 0.06 rad/s 为人工转向阈值，并在松开转向后重置直线参考；现场仍需核对手柄零位和左右符号。跌倒起身 SDK 有 `GetUp()`，但旧 RL 控制器没有自动接入；现场试验先按人工急停和原机恢复流程执行。更详细的 SDK 采集和恢复说明见仓库 `deployment_handoff/T1FootRoom_S43_20261006/README.md`。

## 正在训练的新分支

TorqueRampKeep/Push 仍在 GPU 1/3 续训，截至打包前仅有第 100–300 轮检查点。第 200 轮虽有更低的固定直行力矩请求触限，但平均躯干横滚约 +0.06 rad，未通过原候选的姿态门槛，也未完成随机环境验证，因此**没有加入本次现场包**。后续结果须按相同条件复测，不能凭“最新轮数”替换首测候选。
