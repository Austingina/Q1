# Q1 线路 B 独立网页

手动命令和现场操作清单见 [`q1_line_b_manual_execution.md`](q1_line_b_manual_execution.md)。

线路 B 使用独立入口 `apps.line_b_web` 和独立端口 `8768`，不修改线路 A 的 `apps.dance_web`（默认端口 `8767`）或其收储状态机。

事故前设计状态图为完整行走/舞蹈闭环。2026-11-13 首次网页真机运行在 `GETUP → WALK`
交接后倾倒，因此当前强制使用安全锁定状态图：

```text
LYING（人工标准平趴）→ GETUP → STAND_HOLD（持续跟踪起身末帧）→ LIEDOWN → LYING（阻尼）
```

线路 B 状态机不存在 `FOLDED / UNFOLD / FOLD`。网页和 HTTP API 都不接受折叠动作。
事故锁定期间 API 还会拒绝所有非零移动和舞蹈，不能通过直接调用接口绕过页面禁用状态。

## 离线验收

```bash
infra/bin/q1py -m unittest apps.line_b_web.test_control apps.line_b_web.test_machine
infra/bin/q1py -m apps.line_b_web.sim_check
```

起身到行走的事故专项修复已经接入正式 `LineBMachine`，但缺省不传交接配置，因此状态图
仍停在 `STAND_HOLD`。真机只有两个显式入口可以获得交接 capability：下文独立的一次性
零速验收程序，以及零速验收通过后新增的 `--enable-manual-drive` 人工行走/转向入口。
两者缺省都关闭；舞蹈使用独立 capability，不会随人工行走解锁。复现离线扫参、网页输入
链路和鲁棒性回归：

```bash
infra/bin/q1py -m unittest apps.line_b_web.test_offline_handoff
infra/bin/q1py -m apps.line_b_web.handoff_sim_check
infra/bin/q1py -m apps.line_b_web.web_handoff_sim_check
infra/bin/q1py -m apps.line_b_web.handoff_robustness_check
infra/bin/q1py -m apps.line_b_web.manual_drive_sim_check
```

当前离线候选采用 `10.0 s` smoothstep，将起身策略目标和 PD 增益平滑交给零速度基础策略，
随后再做 `3.0 s` 零速度稳定观察；相邻关节目标变化硬限为 `0.01 rad`。交接期间奇偶帧
交替更新起身/基础策略缓存，因此每个 50 Hz 控制帧最多执行一次 ONNX 推理。开始前检查
策略目标差值，过程中检查直立度、骨盆角速度和关节跟踪误差。

修复后的 `5/6/7/8/9/10/12 s` 扫描中，`5 s` 原始目标变化为约 `0.0116 rad/帧`；`7 s`
虽然标准场景低于 `0.01 rad/帧`，但扰动回归只有 `2/8` 满足该限制；`10 s` 扰动回归为
`8/8` 通过，因此选择 `10 s`。最终 MuJoCo 一次性流程共 `1531` 帧，最大原始/实际目标
步长约 `0.00985 rad`、最低直立度 `0.9987`、最大角速度 `0.0251 rad/s`，完成贴地并进入
`LYING`。该结果只说明离线候选通过，不构成真机放行依据，正式网页缺省仍保持事故锁定。

仿真网页：

```bash
infra/bin/q1py -m apps.line_b_web
```

缺省网页只能执行起身保持和趴下。若要在 Mac 上显示并测试“仿真交接到行走”按钮，必须使用：

```bash
infra/bin/q1py -m apps.line_b_web --host 127.0.0.1 --experimental-sim-handoff
```

实验状态图为：

```text
LYING → GETUP → STAND_HOLD → WALK_HANDOFF（10 s）→ WALK_SETTLE（3 s）→ WALK
```

交接期间网页断联或心跳超时只会把速度归零并继续当前控制状态，显式急停才会立即进入
`DAMPED`。不加实验开关时，网页和 API 仍拒绝交接、移动与舞蹈。

## 一次性零速交接验收（与网页隔离）

`apps.line_b_web.real_handoff_validation` 只执行固定流程：

```text
标准平贴俯卧 → 起身 → WALK_HANDOFF（10 s，零速）
                   → WALK_SETTLE（3 s，零速）→ 贴地 → 阻尼
```

它不起网页服务，不接受手柄，`speed_scale` 固定为 `0`，没有舞蹈转移。每个 50 Hz 控制帧
记录模式、关节位置/速度、IMU、目标、action、PD 增益、速度命令、跟踪误差与交接门禁指标。
缺省只跑 MuJoCo：

```bash
infra/bin/q1py -m apps.line_b_web.real_handoff_validation --backend sim
```

真机只读摆姿检查使用假命令话题，不会建立真实命令话题的 Publisher：

```bash
python3 -m apps.line_b_web.real_handoff_validation --backend real --check-only
```

真机 CPU 推理时延检查同样只使用假命令话题，不调用 `reset()` / `step()`；它不要求机器人
处于标准趴姿，并在任何动作确认之前执行：

```bash
python3 -m apps.line_b_web.real_handoff_validation --backend real --timing-check-only
```

真机动作入口必须同时给出 `--backend real --execute-real-handoff`，还必须在终端手动输入
`线路B零速交接`。少任一条会在创建真机后端前拒绝。该入口只用于专项验收，不代表
网页人工行走或舞蹈自动解锁。

本次修复后的一次性真机零速验收已经通过：共 `1531` 帧，速度命令全程精确为 `0`，
交接段最低直立度 `0.9981`、最大骨盆角速度 `0.0995 rad/s`、最大关节跟踪误差
`0.1054 rad`，最后贴地并进入 `LYING`/阻尼。日志为
`q1/getup/data/real_handoff_repaired_20261113_2203.npz`。

## 一键摆姿检查、人工行走/转向与舞蹈候选

线路 B 新增独立的人工驾驶 capability，必须显式给出 `--enable-manual-drive` 才会出现
“进入人工行走”及六个按住方向按钮。摆姿检查不依赖该 capability：网页在 `LYING / DAMPED`
状态提供“一键检查摆姿”，只读取 IMU 与关节位置，并显示 upright、是否折叠、与起身首帧的
最大关节差及通过/拒绝原因；检查路径不调用 `reset()`、`step()` 或 `set_pd()`。

人工驾驶具有以下硬约束：

- 只有 `WALK` 状态接受非零移动，直接调用 HTTP API 也不能绕过；
- 行走映射直接复用原仓库 `FWD/BACK/TURN`、`ACC` 和 `ramp()`，前进、后退、
  转向不再允许分别设置比例，统一使用原仓库真机示例和开机自启默认
  `--speed-scale 0.5`：前进 `0.30 m/s`、后退 `0.20 m/s`、左右转向 `0.50 rad/s`；
  线速度斜坡 `1.0 m/s²`、转向斜坡 `3.0 rad/s²`、控制周期 `50 Hz`也与原仓库相同。
  不采用通用 CLI 的仿真满速 `0.60/0.40 m/s` 和 `1.00 rad/s`；横移始终为零，
  后退转向组合被拒绝；
- 网页提供前进、后退、左转、右转、前进左转、前进右转六个互斥按钮；单指针锁定会忽略
  第二个同时触摸，防止一次按压串到另一方向；
- 按住时网页每 `100 ms` 刷新命令，松手或超过 `0.25 s` 没有移动命令即归零；
- 网页失焦、隐藏、断开或心跳超过 `1.0 s` 时，前进和转向命令同时归零，但控制器继续
  保持当前状态；
- 只有明确点击红色“立即阻尼急停”才会请求阻尼；
- 页面和服务日志分别记录六种方向的切换、按压次数、持续时间和归零原因；六枚按钮使用双列
  网格并保留底部空间，避免被固定急停栏遮挡；
- 舞蹈使用独立 capability：只有同时给出 `--enable-manual-drive --enable-dance`
  才在网页显示“跳舞（完整动作）”；只接受 `WALK` 状态的请求，首先进入
  `STRAIGHTEN` 将 `vx/wz` 归零并检查骨盆角速度，然后播放默认约 `20.3 s`
  的完整动作，结束后平滑回到零速 `WALK`；
- 线路 A 的折叠状态和接口不进入线路 B。

旧低速候选的 MuJoCo 完整流程
`GETUP → WALK_HANDOFF → WALK_SETTLE → 前进 → 停止 → 后退 → 停止 → LIEDOWN`
已通过；前后命令分别达到 `+0.08/-0.05 m/s`，两次松手最终都归零，人工行走阶段最低
直立度 `0.9996`，最后进入 `LYING`。原仓库真机速度 `+0.30/-0.20 m/s` 候选也已通过
机器人 CPU 上的 40 项无物理单元测试和完整 MuJoCo 位移验收：前进命令达到 `+0.30 m/s`，
末 1 秒平均速度 `+0.3080 m/s`、位移 `+0.869764 m`、最低直立度 `0.994943`；后退命令达到
`-0.20 m/s`，末 1 秒平均速度 `-0.1985 m/s`、位移 `-0.543786 m`、最低直立度 `0.996210`。
两次停止的最终命令均为 `0`，最后完成趴下并进入 `LYING`。这仍是纯仿真结果，不单独构成
提高真机速度的放行依据。

## 一键替换线路 B 舞蹈

舞蹈策略与参考动作是成套训练的，不允许只用新 `motion.npz` 覆盖旧文件。
新舞蹈必须使用完整训练导出包，至少包含：

- `policy.onnx`
- `deploy.yaml`
- `motion.npz`
- `deploy_motion.npz`
- `meta.json`

Mac 上可直接双击项目根目录的 `replace_line_b_dance.command`，然后在弹窗中选择新舞蹈包
文件夹。也可在终端中执行：

```bash
./replace_line_b_dance.command /path/to/new_dance_package
```

如果收到的是 ZIP，终端命令可直接传入 ZIP 路径。只校验而不替换：

```bash
./replace_line_b_dance.command --check-only /path/to/new_dance_package
```

脚本会依次完成：

1. 检查 Q1 关节顺序、50 Hz、观测/ONNX 维度、PD 参数和数值有效性。
2. 核对 `meta.json` 动作哈希、`deploy_motion.npz` 和 `motion.npz`，防止策略/动作串包。
3. 检查舞蹈不超过 `60 s`，且末帧已回到直立静止站姿。
4. 备份旧舞蹈到 `weights/.dance_backups/`，在同一磁盘上原子替换新包，默认保留最近 `5` 份。
5. 自动运行完整 MuJoCo `起身 → 零速交接 → 舞蹈 → 回到 WALK → 趴下` 验收。
6. 任一静态检查失败都不会替换；替换后的 MuJoCo 验收失败会自动恢复旧舞蹈。

该工具默认只修改 Mac 本地候选，不连接、不同步、不启动真机，也不修改线路 A。
本地替换成功后，真机仍需在松软支撑、服务停止的条件下单独同步，再先做无动作验证。

## 真机分阶段顺序

1. APP 选择「低层运控开发」，吊带、独立急停和空旷区域就位。
2. 人工摆到标准平趴姿，先运行 `python3 -m apps.teleop.getup_test --check-only`。
3. 先同步并核对线路 B 独立模块，再用假话题或仿真确认网页、令牌和断联零速保持。
4. 缺省网页只允许摆姿检查、起身、`STAND_HOLD` 和趴下；人工行走/转向必须单独部署、复核
   并显式启用，舞蹈还需再独立给出 `--enable-dance`。
5. 缺省安全锁定启动形式：

   ```bash
   source /opt/ros/humble/setup.bash
   source /robot/software/interface/setup.bash
   export PYTHONPATH=.python:$PYTHONPATH
   python3 -m apps.line_b_web --backend real --enable-real --speed-scale 0.5
   ```

   人工驾驶候选只有在下一轮现场条件再次确认后，才使用：

   ```bash
   python3 -m apps.line_b_web --backend real --enable-real --enable-manual-drive \
     --speed-scale 0.5
   ```

   舞蹈候选只有在完成离线、无动作和现场条件复核后，才使用：

   ```bash
   python3 -m apps.line_b_web --backend real --enable-real --enable-manual-drive \
     --enable-dance --speed-scale 0.5
   ```

程序会打印带随机令牌的地址。通过当前网线从 Mac 访问：

```text
http://10.1.1.100:8768/?token=<启动时打印的令牌>
```

断网、锁屏、关页或心跳超过默认 `1.0 s` 时，`vx/wz` 会同时归零，当前控制状态继续运行；站立保持
不会因网页断联切阻尼。红色按钮会显式请求软件阻尼急停，但不能代替独立硬件急停。

## 操作顺序

1. 状态 `LYING` 时先点击“一键检查摆姿”；只有页面显示检查通过才点击“起身”，等待
   `GETUP → STAND_HOLD`。起身仍会重新执行同一门禁，不能用网页结果绕过。
2. 缺省安全锁定模式下，状态 `STAND_HOLD` 时只能点击“趴下”。
3. 显式人工驾驶候选下，点击“进入人工行走”，等待
   `WALK_HANDOFF（10 s）→ WALK_SETTLE（3 s）→ WALK`；交接期间不要按方向按钮。
4. 只有页面显示 `WALK` 后才使用方向按钮；每次只按一个，松手后同时确认 `vx=0.00`、
   `wz=0.00`，再使用另一方向。不要用网页按钮代替独立硬件急停。
5. 舞蹈候选下，只在页面显示 `WALK` 且现场稳定时点击“跳舞（完整动作）”。
   状态依次为 `STRAIGHTEN → DANCE → WALK`；播放期间普通行走和趴下按钮不可用。
6. 需要结束时先松开方向并确认 `vx/wz=0.00`，再点击“趴下”，完成后进入 `LYING` 并切阻尼。

## 2026-11-13 部署状态

- 新模块已单独部署到机器人 `/home/run/LocoInfra_Q1_Dance/apps/line_b_web/`；没有覆盖线路 A 文件。
- Mac 与机器人端单元测试均通过；与既有线路 A 测试合计 18 项通过。
- 事故前 Mac 与机器人端完整仿真通过，但仿真没有暴露真机策略交接问题，不能再作为行走放行依据。
- 首次网页真机运行完成起身后，在自动切换到 `WALK` 时倾倒；倾倒门禁自动切阻尼，控制器随后人工停止。
- 既有真机日志表明起身末帧保持稳定（upright 最低 `0.9982`，角速度最大 `0.014 rad/s`），基础行走策略首目标与起身目标最大相差 `0.383 rad`；失败运行没有逐帧记录，因此该交接是首要嫌疑而非已证明的唯一根因。
- 当前代码和 API 已事故锁定行走/舞蹈，默认停在 `STAND_HOLD`；在专项交接验收前不得解除。
- 旧版 `5 s` 平滑交接候选当时采用交接中断联阻尼；该行为已在首次人工网页验收暴露为危险设计，
  现已替换为断联零速保持，不能再把旧结果当作当前断联策略的验收依据。
- Mac 本地网页人工验收通过：按钮与状态依次显示 `GETUP → STAND_HOLD → WALK_HANDOFF → WALK_SETTLE → WALK → DANCE → WALK → LIEDOWN → LYING`；第二轮在 `WALK_HANDOFF` 点击网页急停后立即进入 `DAMPED`，页面锁住所有动作，浏览器无警告或错误。
- 离线集成阶段没有连接或同步机器人；真机代码仍是上次部署的事故锁定版本，服务保持停止，未解除行走/舞蹈锁定。
- 随后的机器人连接阶段只做了无电机验证：低层关节/IMU 状态正常、真实命令 Publisher 为 `0`、无控制进程；候选仅复制到 `/tmp` 隔离目录运行后删除，生产文件哈希未变化。
- 首版候选在机器人 CPU 上通过当时的 21 项线路 B 测试、时长扫描和网页输入链路仿真，并在当时选择 `5 s`；3 秒假话题 dry-run 为策略 `49.6 Hz`、假指令 `485.3 Hz`，明确未使用真实电机话题。退出后假话题消失、真实 Publisher 仍为 `0`。该候选随后在真机交接中失败，已被下文的 `10 s` 单推理候选替代。
- 一次性零速验收入口已在 Mac 和机器人 CPU 上完成 MuJoCo 全链路：`1281` 帧，
  `GETUP → STAND_HOLD → WALK_HANDOFF → WALK_SETTLE → WALK → STRAIGHTEN → LIEDOWN → LYING`，
  速度命令全程为 `0`，最大目标步长约 `0.0072 rad`。
- 新入口仅曾复制到机器人 `/tmp` 候选目录；只读预检通过（`upright=+0.033`，首帧最大关节差
  `0.493 rad`，非折叠）。退出后真实命令 Publisher 为 `0`、假话题消失、无残留进程；
  生产网页和事故锁定文件未修改。

## 2026-11-13 一次性真机零速交接结果

- 起身完成，进入 `WALK_HANDOFF` 时 `upright≈0.998`、骨盆角速度约 `0.006 rad/s`。
- 交接进行到第 `176/250` 帧时，角速度增长到 `0.510 rad/s`，超过 `0.500 rad/s`
  门禁；程序立即进入 `DAMPED`，未进入 `WALK_SETTLE`，也未执行贴地。
- 全程速度命令为 `0`，交接期间最低直立度 `0.982`，最大跟踪误差 `0.273 rad`。
  原始目标单帧变化最大 `0.207 rad`，硬限幅后实际下发最大 `0.010 rad`。
- 真机上交接段同时执行起身和基础两个策略，实际只有约 `22.1 Hz`；
  每帧平均 `43.6 ms`、p99 `81.1 ms`，没有达到设计的 `50 Hz`。因此帧数表示的
  “5 s”在真机 CPU 上并不是 5 s 墙钟时间，该实现不得重试。
- 自动阻尼后只读状态为 `upright≈0.248`，机器人处于吊带承重/塌落姿态。
  真实命令 Publisher 为 `0`，无控制进程残留，机器人 `/tmp` 候选已删除。
- 失败日志保存为
  `q1/getup/data/real_handoff_20261113_200143_failed.npz`。下一步必须回到离线修复：
  首先把交接改为每帧最多一次策略推理，并在真机动作前增加推理时延硬门禁。

## 2026-11-13 失败后的离线修复与无电机复测

- 交接已改为奇偶帧交替更新两种策略缓存，每个控制帧最多一次 ONNX 推理；交接时长由
  扰动回归选择为 `10 s`，而不是放宽目标步长、姿态或角速度门禁。
- 新增动作前推理硬门禁：同一会话先完成 `20` 次无动作预热，再按真实 `50 Hz` 节拍分别
  统计 `200` 次起身/基础策略推理；平均值都必须不超过 `15 ms`，p99 都必须不超过
  `18 ms`。按节拍采样避免背靠背 FIFO 推理触发 Linux 每秒 `50 ms` 的实时带宽节流；门禁
  在终端动作确认之前运行，失败时不会调用电机 `reset()` / `step()`。
- 首次门禁正确拒绝：策略进程误绑到 `core 3` 的 A53 小核，测得起身/基础平均约
  `23.0/15.4 ms`、p99 约 `47.6/38.9 ms`。独立绑核基准确认 `core 3` 的完整加载起身模型
  平均/p99 约 `19.9/26.0 ms`，不具备 50 Hz 余量。
- 验收程序默认绑核已修正为 `core 6` 的 A72 大核（ROS 收发继续使用 `core 4/5`）。完整
  假话题门禁复测通过：起身平均/p99 `9.91/14.47 ms`，基础策略 `6.93/9.31 ms`。
- 本轮复测只读取传感器并执行 CPU 推理；真实与假命令话题均无残留 Publisher，无控制进程。
  当前机器人仍是吊带承重下的松软塌落姿态，未再次执行任何动作；生产网页仍维持事故锁定。
- 人工重新摆到标准平趴后，只读门禁通过（`upright=+0.024`、首帧最大误差 `0.358 rad`、
  非折叠）；原位保持最大跟踪偏差 `0.018 rad`、相对起点移动 `0.000 rad`，随后自动阻尼。
- 为排除上次跌倒后的硬件/起身链路异常，又完成一次独立的
  `起身 → 稳定站立 2 s → 趴下 → 阻尼`：站立段最低直立度 `0.99887`、最大角速度
  `0.0142 rad/s`、实际目标跟踪误差最大 `0.0994 rad`。最终只读姿态 `upright=+0.03`、
  首帧最大误差 `0.16 rad`；真实命令 Publisher 和控制进程均为 `0`。逐帧日志为
  `q1/getup/data/real_policy_20261113_213755.npz`。
- 随后动作前时延门禁曾因把 FIFO 推理背靠背跑满 CPU 而出现周期性约 `50 ms` 尖峰；机器人
  没有开始动作。内核参数为 `sched_rt_period_us=1000000`、`sched_rt_runtime_us=950000`，证实
  旧基准触发了每秒约 `50 ms` 的实时带宽节流。门禁已改为按真实 `50 Hz` 节拍采样，三轮
  假话题复测的起身策略 p99 为 `15.43/15.42/15.56 ms`，最大值
  `16.23/16.12/15.84 ms`；基础策略 p99 为 `9.22–10.62 ms`，全部满足原 `15/18 ms`
  阈值。该修复未改内核参数、未放宽门限，42 项线路 A/B 相关测试通过。
- 修复后的最终一次性真机零速交接通过：`1531` 帧、命令速度全零，完成
  `GETUP → WALK_HANDOFF（10 s）→ WALK_SETTLE（3 s）→ LIEDOWN → LYING`；控制进程和
  真实命令 Publisher 均已退出，日志 SHA-256 为
  `ba2f9c94f623ef860e08f04486a2ce3c525f5c937313c317abc54a28db80f9b9`。
- 人工按压前进/后退目前只完成 Mac 离线实现与 MuJoCo 验收；机器人生产网页仍是事故锁定
  版本，尚未进行任何真实前进或后退动作。
- 人工前后候选已只同步到机器人隔离目录
  `/tmp/q1_line_b_manual_drive.8cskbq`，未覆盖正式线路 A/B；六个候选核心文件与 Mac 的
  SHA-256 全部一致，机器人端 37 项离线测试通过。第一次无动作启动暴露 `__main__.py`
  漏导入 `BACK`，在创建状态机时退出、未调用动作；修复后单文件哈希再次核对并重跑 37 项通过。
- 修复后的候选用 `--dry-run --enable-manual-drive` 只绑定机器人本机 `127.0.0.1:18768`：
  状态为 `LYING`、`vx=0`，前进/后退在非 `WALK` 状态均返回 HTTP 400，舞蹈与折叠请求也被
  API 拒绝，零速请求返回 200。真实关节命令 Publisher 始终为 `0`；假话题虽然在服务运行时
  存在 Publisher，但 3 秒观察收到 `0` 条/`0` 字节消息，证明没有调用 `reset/step`。
  服务停止后无残留进程，假话题消失，正式线路 A/B 文件哈希与启动前完全一致。
- 首次人工网页起身进入 `STAND_HOLD` 后，页面失焦/心跳断开触发旧版自动 `DAMP`，机器人在
  吊带保护下软下；日志顺序为 `GETUP → STAND_HOLD → DAMPED`，并非站立姿态门禁失败。
  因此断联语义改为：立即清零行走命令并继续当前状态机，只有显式红色急停才产生 `DAMP`；
  默认心跳容差由 `0.3 s` 增至 `1.0 s`，并记录最近断联原因。
- 修复断联策略后的再次真机验收完成
  `LYING → GETUP → STAND_HOLD → WALK_HANDOFF → WALK_SETTLE → WALK → LIEDOWN → LYING`；
  页面失焦时在 `STAND_HOLD` 保持直立，未进入 `DAMPED`。人工前进产生可见位移，人工后退
  未观察到响应，因此只判定前进通过，后退仍未放行。最终正常趴下并自动阻尼，服务停止后
  控制进程和真实命令 Publisher 均为 `0`。
- 用户要求恢复原 locomotion-infra 真机速度后，Mac 候选已改为 `+0.30/-0.20 m/s`
  （原仓库真机示例/自启默认 `speed-scale=0.5`）。网络恢复后，21 个候选/依赖文件的
  SHA-256 在 Mac 与机器人隔离目录 `/tmp/q1_line_b_mujoco.66lICz` 全部一致，机器人 CPU 上
  40 项无物理单元测试通过。完整 MuJoCo 动态流程随后一次通过：
  `LYING → GETUP → STAND_HOLD → WALK_HANDOFF → WALK_SETTLE → WALK → 前进 → 停止 → 后退
  → 停止 → STRAIGHTEN → LIEDOWN → LYING`；舞蹈请求保持锁定。交接最低直立度 `0.998667`、
  最大角速度 `0.025348 rad/s`、最大跟踪误差 `0.099193 rad`、最终基础策略残差 `0`，动态
  人工行走阶段最低直立度为 `0.994943`。验收耗时 `27.06 s`。开始前和结束后的线路 B 服务
  都是 `inactive/dead`、相关进程为 `0`、真实关节命令 Publisher 为 `0`；生产目录未写入。
- 原速度候选首次短促真机前进由现场观察为“算是正常迈步”，但网页审计同时记录了前进/后退
  各 `2` 次、最近方向为后退、最近一次持续 `4.92 s`，与计划的单次前进 `0.5 s` 不一致，
  因此没有把该次操作判为正式方向输入验收。机器人随后正常趴下，服务和隧道停止，真实关节
  命令 Publisher 回到 `0`。
- 按用户要求新增网页一键摆姿检查，以及原地左/右转和前进左/右转；原仓库真机转向上限为
  `0.50 rad/s`。Mac 与机器人 CPU 上 45 项测试通过，18 个线路 B 文件哈希一致。扩展后的
  完整 MuJoCo 流程一次通过：摆姿检查未下发动作；左/右转 yaw 变化为 `+0.836/-0.813 rad`，
  前进左/右转为 `+0.824/-1.029 rad`；六种移动后的停止命令均为 `vx/wz=0`，移动阶段最低
  直立度 `0.993895`，最终进入 `LYING`。本轮只更新机器人 `/tmp` 隔离候选，未启动真机服务；
  结束审计为服务 `inactive/dead`、线路 B 进程 `0`、真实关节命令 Publisher `0`。
- 行走参数后续收敛为原仓库唯一的 `speed-scale=0.5` 映射，删除线路 B 分别
  覆盖前进/后退/转向比例的入口；数值保持 `+0.30/-0.20 m/s`、`0.50 rad/s`，
  并直接复用原仓库 `1.0 m/s²`、`3.0 rad/s²` 斜坡与 `50 Hz` 控制周期。
  本地 `46` 项测试全部通过；完整 MuJoCo 动态验收再次通过，前进首帧
  `0.02 m/s`、转向首帧 `0.06 rad/s`，六种移动全部达到原仓库真机上限，
  移动阶段最低直立度 `0.993895`，每次松手最终 `vx/wz=0`，最后返回 `LYING`。
- 2026-11-14 将完整舞蹈作为独立 `--enable-dance` capability 接入线路 B 网页候选。
  本次只修改 Mac 本地候选，真机正在充电，未连接、未同步、未启动真机服务。
  本地 `47` 项单元测试通过；完整 MuJoCo 网页链路通过
  `LYING → GETUP → STAND_HOLD → WALK_HANDOFF → WALK_SETTLE → WALK →
  STRAIGHTEN → DANCE → WALK → STRAIGHTEN → LIEDOWN → LYING`，舞蹈时长 `20.3 s`；
  本机临时网页状态同时确认 `manual_drive_enabled=true`、`dance_enabled=true`和
  `disconnect_policy=zero_and_hold`，验收后临时服务已停止。
- 2026-11-14 真机开机后，候选先同步到 `/tmp/q1_line_b_dance_candidate.l2fEfr`：19 个文件
  SHA-256 全部一致，机器人 CPU 上 47 项测试与完整 MuJoCo 舞蹈链路均通过。
  只读摆姿预检为 `upright=0.035`、未折叠、最大起身首帧关节差 `0.288 rad`；
  CPU 时延门禁通过：起身策略 mean/p99 `10.15/14.74 ms`，基础策略
  `6.70/10.00 ms`。
- 无动作验收后，生产 `apps/line_b_web` 已备份到
  `.codex_backups/20261114_0126_line_b_dance/` 并同步新候选；生产目录 19/19 文件哈希一致、
  47 项测试通过。最后用假命令话题启动真机网页候选，确认
  `manual_drive_enabled=true`、`dance_enabled=true`、`dance_duration_s=20.3` 与
  `disconnect_policy=zero_and_hold`。全程真实关节命令 Publisher 为 `0`；退出后假话题消失、
  无控制进程残留。
- 2026-11-14 新增 `replace_line_b_dance.command` 完整舞蹈包一键替换工具；已在
  `/tmp` 隔离目标完成一次真实原子安装和完整 MuJoCo 验收，另用单元测试
  确认仿真验收失败时会恢复旧包。安装器与线路 B 累计 `49` 项测试通过；
  本轮未连接或同步真机。
