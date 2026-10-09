# HRT T1 步态项目

**当前入口：[T1 现场控制器预备包](field_next/README.md)。**这里包含已经整理好的按键状态机、接口代码、S55Warm800 保底模型和现场执行步骤。S57 仍在训练和评估，最终部署模型尚未定稿；S57 选定后会重新导出并生成最终包。10 月 6 日的 TorqueTrial 包只用于历史追溯。

`trained_policies/` 保留训练候选和评估视频；`deployment_handoff/`、`train_kit/`、`robot_code/` 保留历史交接与上游对照。进入机器人之前，先从当前入口核对版本和模型哈希，不从历史 README 选择程序。
