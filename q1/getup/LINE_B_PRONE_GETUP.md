# 线路 B：人工标准平贴姿 → 俯卧起身

这条线路独立于深折叠收储线路，直接使用已有的 `weights/getup_vpd_ft`。程序会明确拒绝深折叠姿、非俯卧姿和与参考首帧偏差过大的人工摆姿。

标准首姿可直接参考 `weights/getup_vpd_ft/play_getup_facedown.mp4` 的前 1 s：躯干朝下，双腿在躯干后方而非收到两侧，双侧 hip pitch 约 `-0.12 rad`、hip yaw 接近 `0 rad`、knee 约 `0.24 rad`，双肘约 `1.50 rad`。人工只需摆到接近位置，程序会用编码器做最终判定。

## 软件验收

```bash
python3 -m apps.teleop.prone_getup_sim_check
```

通过条件是只经过 `LYING → GETUP → WALK`，不经过 `FOLDED / UNFOLD / FOLD`，且起身后的骨盆高度与直立度达标。

## 真机执行顺序

在 Q1 运控板上，先 source ROS 2 Humble 和 PrimeBot SDK，APP 选择「低层运控开发」。全程使用吊带和独立急停，充电线、网线与人员离开机器人动作范围。

1. 关闭位置控制或保持阻尼，由人工将机器人摆到平贴姿，不要把双髋收到躯干两侧的深折叠位置。
2. 只读预检；该命令不发布电机指令：

   ```bash
   python3 -m apps.teleop.getup_test --check-only
   ```

3. 预检通过后，先做保持测试：3 s 复位插值后再原位保持 3 s，确认关节不跳变：

   ```bash
   python3 -m apps.teleop.getup_test --hold-only
   ```

4. 再次人工复核场地、吊带和急停，执行起身。起身后会站住，回车后按参考轨迹贴下并切阻尼：

   ```bash
   python3 -m apps.teleop.getup_test
   ```

`Ctrl-C` 会切阻尼。每次真机运行都会将关节、IMU、策略输出与阶段记录到 `q1/getup/data/real_policy_<时刻>.npz`。

起身到基础策略的交接不再使用网页首次验证。独立的一次性零速验收入口是：

```bash
# 缺省 MuJoCo；自动起身、10 s 零速交接、3 s 观察、贴地、阻尼
python3 -m apps.line_b_web.real_handoff_validation --backend sim

# 真机只读预检；不发布真实电机指令
python3 -m apps.line_b_web.real_handoff_validation --backend real --check-only

# 真机 CPU 时延门禁；使用假话题，不调用电机 reset/step
python3 -m apps.line_b_web.real_handoff_validation --backend real --timing-check-only
```

它无网页、无行走命令、无舞蹈状态，并保存全部 50 Hz 控制帧。真机下发还需要专用
`--execute-real-handoff` 参数和终端固定短语；不应当把该专项验收入口当作日常控制器。

完整的“起身 → 低速前后行走 → 跳舞 → 再行走 → 趴下”使用独立的
`apps.line_b_web`（默认端口 `8768`），说明见 `docs/q1_line_b_web.md`。它不调用线路 A 的折叠状态。

## 2026-11-13 真机验收

- 人工摆到标准平贴姿后，只读预检通过：`upright=+0.03`，与起身首帧最大单关节差 `0.47 rad < 0.70 rad`。
- 保持测试通过：最大跟踪偏差 `0.018 rad`，相对起点最大移动 `0.000 rad`，随后自动切阻尼。
- 完整闭环 `起身 → 站立 5 s → 趴下 → 阻尼` 通过；站立阶段 `upright=+1.00`，角速度最低记录为 `0.00 rad/s`。
- 策略段共 1128 步、22.6 s，7 次超拍；推理 mean `11.4 ms`、p99 `17.0 ms`。
- 结束后只读核验：机器人回到标准趴姿（`upright=+0.01`），真实命令话题 Publisher 为 0。
- 真机日志：`/home/run/LocoInfra_Q1_Dance/q1/getup/data/real_policy_20261113_182027.npz`。
