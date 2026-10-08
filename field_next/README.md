# T1 下一次现场调试入口

**状态（2026-10-08）：离线修订版，尚未重新部署和实机复测。**上一次在新 T1 上运行的 S46 模型能前进，但停车、转向、待命和按键重复进入没有通过连续验收。本目录的程序包用于出发前代码核对，不能当成“已可一键上机”的版本。

- [事故复盘、目标状态机、训练修改和一小时现场验收](REVIEW_20261008.md)
- [S46 当前部署源码与模型压缩包](S46_OFFLINE_CANDIDATE_20261008.zip)
- [压缩包 SHA256](S46_OFFLINE_CANDIDATE_20261008.sha256)
- [Booster 原厂手柄键位](https://docs.booster.tech/zh-CN/docs/product-manual/t1/basic-operations/joystick-control/)

当前控制器仍是 **X 进入 Custom → A 启用 S46 → 摇杆驾驶 → B 停止并回原厂 Prepare**。松开摇杆不应自动退出 Custom；B 正常退出后，本机修订版会重新等待下一次 X，不再依赖 SSH 手动重启。组合键 `LT/RT+X/A/B` 不触发这三个自定义动作。独立低重心待命、单键进入即驾驶、B 第一次回待命、后退和转向稳定性仍属**下一版目标**，不能把状态机图误认为已经实现。

模型为 `StrideTorqueB S46` 第 2200 轮，TorchScript SHA256：`80af933e2034601a51e3ff33e151df973d0b3540f531c2110984957632f22639`。80 维观察、21 动作，适配器映射为 23 个关节目标。压缩包沿用 [Booster Deploy](https://github.com/BoosterRobotics/booster_deploy) 的 Apache 2.0 许可与项目框架，包含 `tasks/hrt_s46`、T1 所需的官方步态模块、修改过的控制器和依赖清单；无关机型和舞蹈模型已排除。

旧的 `deployment_handoff/`、`trained_policies/`、`train_kit/` 和 `robot_code/` 是历史证据或上游参考，不是下一次现场的操作入口。现场接入必须先核对实际机器、固件、急停、原厂模式与板上服务版本，避免把旧机器或旧程序当成这份候选。
