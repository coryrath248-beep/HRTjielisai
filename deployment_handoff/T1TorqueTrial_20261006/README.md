# T1 TorqueTrial：只读试验包

**这份包不能让机器人行走。** `tools/` 只订阅状态、离线计算模型动作；不切换模式、不发电机命令。不要把 `.pt` 覆盖旧模型，也不要因使用本包进入 `kCustom` 或 `kDamping`。现场新增的 `deploy_torque.py`、`joint_mapping_test.py` 不在本包内，不作为本包的测试步骤。

## 现场只做这件事

确认机器人和原厂控制已恢复正常后，在机器人上解压 ZIP，进入 `deployment_handoff/T1TorqueTrial_20261006/`，用实际 SDK 网卡采集 60 秒状态：

```bash
python3 tools/probe.py --net eth0 --seconds 60 --output /tmp/t1_probe.jsonl
```

2026-10-06 那台机器使用 `eth0` 有效，`127.0.0.1` 未收到状态；换机器时先确认网卡。采集结束应看到 `state`、`odom` 均大于零且 `bad_count/callback_error` 为零。把日志交给开发者即可；**不需要在机器人上测试切换模式**。`tools/inspect_capture.py` 和 `tools/replay.py` 可离线分析，原始 profile 的现场核对项都是 `false`。

## 包中的两个模型

| 仿真候选 | 1.6 m/s 指令下 60 秒实速 | 横漂 | 力矩请求触限 | 随机环境渐增起步存活 |
| --- | ---: | ---: | ---: | ---: |
| Mild 200（首选评估） | 1.485 m/s | −0.019 m | 5.4% | 125/128 |
| Firm 800（速度备选） | 1.518 m/s | +0.010 m | 9.4% | 123/128 |

这些都是仿真结果，不是实机性能。`models/` 有 80→21 TorchScript 和训练 YAML，`evidence/` 有原始 JSON，`videos/` 有正面、侧面视频。`python3 verify_package.py` 只校验模型哈希和输入输出维度；校验通过**不代表可以上机行走**。

## 真正部署前还缺什么

开发者需要把 23 路关节状态、IMU、里程计和指令拼成模型要求的 80 维观察；把 21 个臂／腰／腿动作按 YAML 映射到电机目标；核对踝部坐标；建立从站姿交接开始就持续有效的低层发令和停机路径。这段实机控制器**尚未交付或验证**。原厂站立／行走程序和本包分开保存。
