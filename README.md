# HRT T1 行走策略

## 现场只看这一份

[当前试验包：T1TorqueTrial_20261006](deployment_handoff/T1TorqueTrial_20261006/README.md) · [下载 ZIP](deployment_handoff/T1TorqueTrial_20261006.zip)

包内有 TorqueMild 200 和 TorqueFirm 800 的模型、配置、仿真结果及视频。**它是只读采集和影子推理包，不是可驱动机器人行走的程序。**确认机器人和原厂控制已恢复正常后，现场只需采集状态并把日志带回；不需要为此切换 `kCustom`、`kDamping`，也不需要替换原机 `.pt`。

2026-10-06 的现场新增了 `deploy_torque.py`、`joint_mapping_test.py`；它们**不属于此仓库交付包**。19:26 的模式调用返回 100，不能据此认定已进入 `kCustom`；18:41 的逐关节日志也未证明映射通过。查明原因前不要用这些脚本再试模式或电机命令。

要让新模型真正行走，还缺经过现场核对的 **80 维观察 → 21 动作 → 23 路电机**适配与连续发令控制器。旧 `train_kit/deploy/` 是 47 维、12 动作的历史程序，只作原机软件追溯，不能直接加载新权重。

## 历史资料

[`T1FootRoom_S43_20261006`](deployment_handoff/T1FootRoom_S43_20261006/) 和 [`T1PathStride_seed42_20261005`](deployment_handoff/T1PathStride_seed42_20261005/) 是先前交接记录；`trained_policies/` 是仿真候选档案。它们都不是本次现场操作入口。`train_kit/` 和 `robot_code/` 保留上游代码供开发对照。
