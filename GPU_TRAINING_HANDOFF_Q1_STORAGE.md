# Q1 折叠 / 展开技能：NVIDIA GPU 训练机交接说明

> 请从本文开始。本包是当前工作树快照，不包含 `.git` 历史。
> 全流程先在 Linux + NVIDIA GPU 上离线完成，不需要连接 Q1。

## 1. 到底训练什么

这次**不训练 Velocity 走路，不重训舞蹈，也不重训现有起身**。现有三部分继续复用：

- `weights/dual_v3b`：LocoInfra 已有的站立 / Velocity 走路基础模型；
- `weights/q1_smooth_dance`：已训练的舞蹈 ONNX；
- `weights/getup_vpd_ft`：已有的平趴起身 / 趴下技能。

本次只训一个包含两段参考动作的闭环跟踪策略：

1. `unfold_facedown`：深度折叠收储姿势 → 平趴姿势；
2. `fold_facedown`：平趴姿势 → 深度折叠收储姿势。

两段各 640 帧、50 Hz、12.78 秒，共用一个 checkpoint 训练。验收后把同一个 checkpoint 按两个方向分别导出为：

- `weights/q1_unfold/`
- `weights/q1_fold/`

这样底层状态机才能完整走通：

```text
开机：FOLDED → UNFOLD → GETUP → WALK / DANCE
关机：WALK → LIEDOWN → FOLD → FOLDED
```

`q1/storage/motions/*.npz` 是训练参考，**不是可直接下发电机的轨迹**。

## 2. 训练包里已提供什么

| 内容 | 路径 | 用途 |
|---|---|---|
| 两段训练参考 | `q1/storage/motions/` | 直接作为 SuperMimic 训练数据 |
| 原始参考与哈希 | `q1/storage/source/` | 数据溯源，不作电机输入 |
| 参考构建器 | `q1/storage/build_storage_refs.py` | 可重建 NPZ，核对可重现性 |
| 训练启动器 | `infra/SuperMimic-Q1/scripts/train/storage.sh` | 两段动作共同训练 |
| 验收 / 渲染 / 导出 | `infra/SuperMimic-Q1/scripts/` | 生成报告、视频、ONNX 包 |
| Q1 URDF / MuJoCo 模型 | `q1/q1_v3/` | 仿真机器人模型 |
| 现有基础 / 起身 / 舞蹈包 | `weights/` | 合同比对及最终整合 |
| 底层状态机和手机网页 | `apps/` | 最终离线联调与真机接入 |

不需要再从机器人复制数据。训练机需要联网下载 Python / CUDA 依赖和三个固定 commit 的外部仓库；固定 commit 已写入 `infra/setup.sh`。

## 3. GPU 训练机要求

- Linux x86_64；NVIDIA GPU 和可用的 CUDA 驱动，`nvidia-smi` 正常；
- 建议显存 24 GB 或更高，默认为 4096 并行环境；显存不足可明确改为 2048；
- 建议剩余磁盘空间至少 30 GB；
- `git`、C/C++ 构建工具、`ffmpeg`、[`uv`](https://docs.astral.sh/uv/)；
- 训练环境使用 Python 3.12，数据 / 部署检查环境使用 Python 3.11，都由 `uv` 隔离。

先保存机器信息，交付时一并返回：

```bash
nvidia-smi > GPU_INFO.txt
uname -a > SYSTEM_INFO.txt
```

## 4. 解压、完整性和环境安装

```bash
unzip Q1_storage_GPU_handoff_2026-11-13.zip
cd Q1_storage_GPU_handoff_2026-11-13
sha256sum -c SHA256SUMS.txt
cd LocoInfra_Q1_Dance
```

安装数据 / 部署环境和固定的外部仓库：

```bash
bash infra/setup.sh
```

安装 GPU 训练环境：

```bash
cd infra/SuperMimic-Q1
uv sync --group dev
uv pip install --python .venv/bin/python onnx onnxruntime
cd ../..
```

验证 CUDA 可用：

```bash
infra/SuperMimic-Q1/.venv/bin/python -c 'import torch; print(torch.__version__, torch.cuda.is_available(), torch.cuda.get_device_name(0))'
```

`torch.cuda.is_available()` 必须为 `True`。

## 5. 预检和可选的参考重建

先测当前附带的参考和状态机：

```bash
infra/bin/q1py -m unittest \
  q1.storage.test_storage_refs \
  apps.teleop.test_machine_storage \
  apps.dance_web.test_package_compat
```

如果需要证明数据可重现，可重建两段参考，然后重跑上述测试：

```bash
cd infra/SuperMimic-Q1
SUPERMIMIC_ROBOT=q1 .venv/bin/python ../../q1/storage/build_storage_refs.py
cd ../..
```

重建后 `q1/storage/motions/manifest.json` 中的源文件哈希必须与包内一致。

## 6. 训练

在正式训练前可做一个短时冒烟训练；它只证明环境能跑，**不是可交付权重**：

```bash
cd infra/SuperMimic-Q1
scripts/train/storage.sh q1_storage_smoke \
  --env.num-envs 64 \
  --schedule.max-hours 0.05
```

正式训练，默认 4096 环境，最长 4 小时：

```bash
scripts/train/storage.sh q1_storage_v1 2>&1 | tee logs/q1_storage_v1_train.log
```

只有确认 OOM 时才降为 2048，并在交付说明里记录变更：

```bash
scripts/train/storage.sh q1_storage_v1 \
  --env.num-envs 2048 \
  --schedule.max-hours 8 \
  2>&1 | tee logs/q1_storage_v1_train.log
```

不应该因为训练命令结束就直接导出；必须先完成下一节验收。主 checkpoint 预期位置：

```text
infra/SuperMimic-Q1/logs/q1_storage_v1/ckpt_latest.pt
```

## 7. 仿真验收和可视化

运行完整验收套件：确定性回放、域随机、外力扰动、求解器变化和视频。

```bash
cd infra/SuperMimic-Q1
MUJOCO_GL=egl SUPERMIMIC_ROBOT=q1 .venv/bin/python scripts/eval_report.py \
  --ckpt logs/q1_storage_v1/ckpt_latest.pt \
  --npz-dir ../../q1/storage/motions \
  --clips unfold_facedown fold_facedown \
  --out-dir logs/q1_storage_v1/eval \
  --dr-episodes 1000 \
  --push-per-speed 64 \
  --device cuda:0
```

通过标准：

- `logs/q1_storage_v1/eval/report.json` 的顶层 `passed` 必须为 `true`；
- 报告中所有带判定的 `checks[].passed` 都必须为 `true`；
- 查看 `play_unfold_facedown.mp4` 和 `play_fold_facedown.mp4`：方向正确，无穿地、翻滚、异常撞地或关节猛跳。

如果 `passed=false`，请保留报告和视频，返回失败证据，**不得标记为可部署**。

## 8. 用同一 checkpoint 导出两个方向

```bash
SUPERMIMIC_ROBOT=q1 .venv/bin/python scripts/export_package.py \
  --ckpt logs/q1_storage_v1/ckpt_latest.pt \
  --motion-npz ../../q1/storage/motions/unfold_facedown.npz \
  --eval-dir logs/q1_storage_v1/eval \
  --out ../../weights/q1_unfold \
  --job q1_storage_v1_unfold

SUPERMIMIC_ROBOT=q1 .venv/bin/python scripts/export_package.py \
  --ckpt logs/q1_storage_v1/ckpt_latest.pt \
  --motion-npz ../../q1/storage/motions/fold_facedown.npz \
  --eval-dir logs/q1_storage_v1/eval \
  --out ../../weights/q1_fold \
  --job q1_storage_v1_fold
```

导出器会自动校验 PyTorch 与 ONNX 输出，最大误差必须小于 `1e-5`。每个包至少应有：

```text
policy.onnx
deploy.yaml
motion.npz
deploy_motion.npz
ckpt.pt
meta.json
report.json
report.html
play_unfold_facedown.mp4
play_fold_facedown.mp4
```

## 9. 用纯部署路径再验两次

这一步用 ONNX Runtime + CPU MuJoCo 走最接近实际部署的路径：

```bash
SUPERMIMIC_ROBOT=q1 .venv/bin/python scripts/deploy_sim_check.py \
  --package ../../weights/q1_unfold --device cuda:0

SUPERMIMIC_ROBOT=q1 .venv/bin/python scripts/deploy_sim_check.py \
  --package ../../weights/q1_fold --device cuda:0

cd ../..
infra/bin/q1py q1/storage/validate_trained_packages.py
```

最后一条必须输出：

```text
PACKAGE VALIDATION: PASS (offline only; real-robot gates are still required)
```

它会检查方向、首尾姿态、与现有起身 / 趴下的交接、基础模型合同、ONNX 有限输出、完整验收报告及部署仿真结果。

## 10. 离线整链联调

从深度折叠姿势启动仿真手机网页：

```bash
infra/bin/q1py -m apps.dance_web \
  --unfold-skill weights/q1_unfold \
  --fold-skill weights/q1_fold \
  --sim-start-folded
```

用浏览器打开终端显示的本机地址，在仿真中分别核对：

1. 起身：`FOLDED → UNFOLD → GETUP → WALK`；
2. 低速走路与停稳；
3. 舞蹈进入和退出；
4. “趴下”只到平趴；
5. “折叠 / 准备关机”到深度折叠并保持。

请录屏或保存整链联调证据。

## 11. 训练机必须交回的内容

请整理成 `Q1_STORAGE_TRAINED_RESULTS_<date>.tar.gz`，至少包含：

```text
weights/q1_unfold/
weights/q1_fold/
infra/SuperMimic-Q1/logs/q1_storage_v1/ckpt_latest.pt
infra/SuperMimic-Q1/logs/q1_storage_v1/eval/
infra/SuperMimic-Q1/logs/q1_storage_v1_train.log
GPU_INFO.txt
SYSTEM_INFO.txt
TRAINING_NOTES.md
```

`TRAINING_NOTES.md` 请记录：

- GPU 型号、驱动、CUDA / PyTorch 版本；
- 实际命令、并行环境数、训练时长、checkpoint iteration；
- 是否有 OOM / NaN / 中断续训；
- `report.json` 的总结果；
- 两个 `deploy_sim_check.json` 结果；
- `validate_trained_packages.py` 的完整输出；
- 人工视频审查结论和任何异常。

可在仓库根目录执行：

```bash
tar -czf Q1_STORAGE_TRAINED_RESULTS_$(date +%F).tar.gz \
  weights/q1_unfold weights/q1_fold \
  infra/SuperMimic-Q1/logs/q1_storage_v1/ckpt_latest.pt \
  infra/SuperMimic-Q1/logs/q1_storage_v1/eval \
  infra/SuperMimic-Q1/logs/q1_storage_v1_train.log \
  GPU_INFO.txt SYSTEM_INFO.txt TRAINING_NOTES.md
```

## 12. 真机部署边界

GPU 训练机可以完成训练、全部仿真验收、导出和离线整合，但**仿真 PASS 不等于允许立即上真机**。

权重回传后，真机依次必须完成：

1. 本机包合同验证；
2. 假话题 `dry-run`，确认不向电机话题发送；
3. 吊带 + 现场独立急停下的低层保持；
4. 先单段 UNFOLD，再单段 GETUP，然后测完整起身链；
5. 先单段 LIEDOWN，再单段 FOLD，然后测准备关机链；
6. 最后才开放走路、舞蹈和完整手机网页。

不应让无现场安全条件的训练机远程执行真实电机命令。最终真机入口形式是：

```bash
infra/bin/q1py -m apps.dance_web \
  --backend real --enable-real --safety-profile full \
  --unfold-skill weights/q1_unfold \
  --fold-skill weights/q1_fold
```

但只能在上述门禁全部完成后，由现场操作员执行。
