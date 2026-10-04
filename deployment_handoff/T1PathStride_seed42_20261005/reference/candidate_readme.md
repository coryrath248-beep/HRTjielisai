# T1 PathStride 第 800 轮候选存档（2026-10-04）

## 权重与来源

- 主候选：`seed42/model_800.pth`，SHA256 `3cfa7d4773981481b8ef73485712e8b83de8fda1dabbcc94a8fac228001cd93b`。服务器原件：`/home/t4/booster_t1_fullbody_20261003/booster_gym/logs/T1FullBody16PathStride_2026-10-04-03-21-43_462480/nn/model_800.pth`。
- 复核种子：`seed43/model_800.pth`，SHA256 `633262fe5b6bcb5593e2737ba3189d14230d233370c9ea89c828f6dbdcc94edf`。服务器原件：`/home/t4/booster_t1_fullbody_20261003/booster_gym/logs/T1FullBody16PathStrideSeed43_2026-10-04-03-21-44_462481/nn/model_800.pth`。
- 两组各自的训练配置和 TensorBoard 原始事件文件保存在 `seed42/`、`seed43/`；全部中间检查点、运行日志和录像原件仍在服务器。两组均从 `SafeSpeed90` 种子 42 的第 700 轮权重出发，只继承模型参数，随机种子分别为 42/43。
- 两份第 800 轮权重及对应配置、固定命令评估已推送到 [项目 GitHub 仓库](https://github.com/coryrath248-beep/HRTjielisai/tree/main/trained_policies/T1FullBody16PathStride_20261004)，权重提交 `17a674e`，诊断补充提交 `06fc1aa`；仓库现有部署接口仍是旧 47 维、12 动作版本。

## 固定指令结果

主候选在 MuJoCo 固定 1.6 m/s 指令下，30 秒存活、机体系平均前速 1.447 m/s、世界坐标横漂 -0.156 m、末端偏航 -0.007 rad、每步基座前进 0.798 m、步频 1.799 Hz。至少一个关节的限幅前力矩请求超限的物理步占 14.6%，关节×步占 0.7%，URDF 关节速度触限步占 0%。60 秒复测同样存活，前速 1.447 m/s、横漂 -0.154 m、力矩请求触限 14.5%、关节速度触限 0%。1.8 m/s 指令的 30 秒复测存活、实速 1.547 m/s、横漂 -0.155 m、力矩请求触限 17.7%、关节速度触限 0%。复核种子在 1.6 m/s 指令下的 30 秒实速 1.452 m/s、横漂 -0.198 m、力矩请求触限 13.6%，但关节速度触限步占约 0.61%。

Isaac Gym 训练仿真器的 64 环境、30 秒固定 1.6 m/s 复测：主候选存活率 62/64（96.9%）、存活者后半程平均前速 1.471 m/s、横漂绝对值均值 0.142 m、90 分位 0.297 m、后半程力矩请求触限均值 27.2%；复核种子存活率 59/64（92.2%）、后半程前速 1.529 m/s、横漂绝对值均值 0.092 m、力矩请求触限 32.1%。主候选在 1.8 m/s 指令的 64 环境存活率 56/64（87.5%）、后半程前速 1.604 m/s、力矩请求触限 34.7%。这些值反映仿真随机初始位置和扰动，不是真机测试。

## 视频与评估

- 主视频：`../training_videos/T1FullBody16PathStride_model800_vx16_30s.mp4`（30 秒，MuJoCo，固定目标 1.6 m/s）。录像脚本实测前速 1.408 m/s，与独立评估器 1.447 m/s 略有差异；两者均显示完整 30 秒和小横漂。
- 同一权重、同一指令的正面与侧面 30 秒录像：`../training_videos/T1FullBody16PathStride_model800_vx16_front_30s.mp4`、`../training_videos/T1FullBody16PathStride_model800_vx16_side_30s.mp4`；同步双视角版：`../training_videos/T1FullBody16PathStride_model800_vx16_front_side_30s.mp4`。MuJoCo 相机方位角分别为 180° 和 90°，录像脚本两次均测得走满 30 秒、机体系前速 1.408 m/s、横漂 -0.162 m。
- 12 秒视频：`../training_videos/T1FullBody16PathStride_model800_vx16.mp4`；复核种子视频：`../training_videos/T1FullBody16PathStrideSeed43_model800_vx16.mp4`。
- 逐轮、30/60 秒和多命令评估：`../evaluations/T1FullBody16PathStride*json`。
- 新增 30 秒左右步态诊断：`../evaluations/T1FullBody16PathStride_model800_symmetry_vx16_30s.json` 和 `../evaluations/T1FullBody16PathStrideSeed43_model800_symmetry_vx16_30s.json`。两组相位对齐的左右腿关节角 RMS 差分别为 0.3327/0.3343 rad，主要集中于髋偏航；左右脚平均周期和步长接近。种子 42 的 MuJoCo 力矩请求触限主要见左/右踝俯仰 6.26%/4.89% 的关节物理步。

## 边界与下一步

这一版是当前最好的仿真候选，尚未真机部署。actor 增加了相对直线起点的横向偏差和航向误差两维输入，实机控制端目前尚未核对能否以相同坐标系、频率和延迟提供这些量。正负原地转向在多命令回放中明显不对称，1.8 m/s 的多环境存活率和力矩触限也不满足直接提速部署。下一步先核对真机状态估计、动作关节映射和电机裕量，再做安全条件下的低速验收。
