# Q1 LocoInfra 假话题部署记录（2026-11-13）

## 已完成

- 部署目录：`/home/run/LocoInfra_Q1_Dance`
- 私有离线 Python 依赖：`/home/run/LocoInfra_Q1_Dance/.python`
- 已验证导入：MuJoCo 3.3.7、ONNX Runtime 1.23.2、NumPy 1.26.4、ROS 2 `rclpy`、`aimdk_msgs`
- 已加入自动假话题检查：`apps/dance_web/q1_dry_run.sh`
- 假命令话题硬编码为 `/kabutack/dry_run/joint/command`，真实电机话题是 `/aima/hal/joint/command`。
- 已修正原厂 MC 进程检测：当前固件的进程 comm 为 `mc_main`，可执行文件为 `mc_app_main`。
- 预检失败时已验证无 `dry_run_check` / `spawn_main` / `resource_tracker` 残留进程。

## 当前门禁

Q1 仍在原厂标准模式，`mc_main` 在运行，ROS graph 没有：

- `/aima/hal/joint/state` (`aimdk_msgs/msg/JointStateArray`)
- `/aima/hal/imu/state` (`sensor_msgs/msg/Imu`)

因此 1 秒预检在启动实时收发子进程前安全停止，没有进入策略循环，也没有发布关节指令。

## 下一步

1. 操作员在官方 APP 选择“低层运控开发”，按当前固件的厂商流程重启。
2. 重启后先只读核对 `mc_main` 已停止、两个状态话题已出现。
3. 运行 3 秒假话题 dry-run，验证传感器连续性、ONNX 50 Hz 推理、假指令≥400 Hz 和退出清理。
4. 只在现场明确确认吊带和独立急停就位后，才可进入真实电机话题的站立保持和低速行走验证。

## 重启后结果（14:20）

- APP 低层运控开发已生效：`mc_main` / `mc_app_main` 未运行。
- `/aima/hal/joint/state` 与 `/aima/hal/imu/state` 已出现并持续发布。
- 3 秒完整 dry-run 通过：
  - 策略：150 帧，49.5 Hz；
  - 关节状态：617 帧；
  - 假指令：1476 帧，487.2 Hz；
  - ONNX 推理：mean 9.345 ms，p99 22.579 ms；
  - 控制超时计数：8 / 150；
  - 指令话题：`/kabutack/dry_run/joint/command`；
  - 真实电机话题未使用，退出无残留进程。
- 当时姿态 `upright = 0.1955`，机器人仍接近趴卧。
- 真实命令话题当时 Publisher 0、Subscriber 1（`device_manager_node`）。
- 电池采样：26%，44.7 V，43.3 °C。真机起身/行走前应先继续充电。

## 充电期间软件安全加固

- 新增 `getup` / `walk` / `full` 三档断联保护，真实电机网页入口禁止关闭该保护。
- 网页自动维持控制心跳；手机锁屏、断网、关页或默认 0.3 秒心跳超时后，机器人端产生一次 `DAMP` 事件并清零速度。
- 页面底部常驻红色阻尼急停；触发后前端锁住动作，必须显式重新连接才能继续。
- `getup` 只开放起身/站立/阻尼；`walk` 只增加前后行走，前进上限 0.08 m/s。
- 控制逻辑、HTTP 令牌与断联保护回归测试共 10 项已通过；测试未连接机器人，未发布任何电机指令。

## 充电完成后恢复（15:05–15:09）

- SSH 已恢复，机器人本次开机时间 `2026-11-13 15:03:59 +08:00`。
- `mc_main` / `mc_app_main` 与旧网页控制器均未运行；关节、IMU 低层状态话题正常。
- 机器人热点为 `Q1-2610001106`，网页热点侧地址为 `192.168.88.88`；`p2p0` 工作在 AP 模式。
- 新版网页的五个文件经 SHA256 一致性校验后同步到 `/home/run/LocoInfra_Q1_Dance/apps/dance_web/`；旧版备份位于 `.codex_backups/20261113_1508_web_v1/`。
- 机器人本机 10 项网页控制测试全部通过。
- 3 秒假话题复测通过：策略 49.2 Hz、假指令 484.1 Hz、推理 mean 8.554 ms / p99 22.428 ms、真实电机话题未使用、`upright=0.195`。
- 曾在 8767 端口启动 `--dry-run --safety-profile getup` 网页供手机验证；随后已停止。

## 首次真实起身与折叠姿态诊断（15:20–15:52）

- 在用户确认充电线移开、吊带/独立急停/空旷场地就位后，真实网页以 `getup` 配置启动。
- 点击一次起身后，`PrimeBotBackend` 在进入策略前触发“关节实际位置偏离指令过大”，自动切阻尼；机器人未产生可见运动。
- 真实关节状态与 `getup_facedown` 首帧最大差为 2.79 rad（右髋 pitch）：当前双侧 hip pitch 约 -2.91 rad、hip yaw 约 ±1.47 rad，是收储式折叠趴卧；现有平趴参考首帧的 hip pitch 约 -0.12 rad、hip yaw 约 ±0.1 rad。
- 全部 455 帧现有起身参考中也没有接近该折叠姿态的帧：最近帧 RMS 仍为 0.98 rad，最大单关节差 2.52 rad。
- 官方 SDK 说明低层运控开发会关闭本体高层运控。实机低层模式虽能发现 `SetMcAction/GetMcAction` 名称，但只读 `GetMcAction` 没有可用响应端，因此不能在同一低层模式直接借用原生 `STAND_UP`。
- 真实网页已停止，8767 端口关闭，机器人保持阻尼；未再次尝试真实起身。

## 折叠起身路线的软件进展

- 新增 `apps/teleop/lying_pose.py`：用双侧 hip pitch/yaw 区分折叠趴卧与平趴。
- 状态机新增可选 `FOLDED_GETUP` 技能入口；未安装该技能时，折叠姿态会停留在 `LYING` 并明确拒绝，禁止进入旧策略首帧插值。
- `getup_test.py` 同样增加折叠姿态门禁，避免命令行测试重复触发后端偏差故障。
- 新增 `capture_native_folded_getup.py`：在保留原厂 MC 的开发模式下，通过公开 `SetMcAction(STAND_UP)` 记录关节、IMU、`/odom` 与 `/odom_torso`，且必须显式传入 `--execute-native-action` 并现场输入 `STAND`。
- 新增 `native_capture_to_motion.py`：成功记录转换为 50 Hz SuperMimic 参考；`/odom` 有效覆盖不足 95% 时拒绝生成，不猜测根轨迹。
- 机器人端16项姿态识别、网页控制和权重契约测试全部通过。
- 真机关节/IMU + 假命令话题验证通过：网页收到起身请求后保持 `LYING`，返回“检测到折叠趴卧：当前平趴起身包不适用，折叠起身技能尚未安装”；未调用 `reset()`、未进入策略、未写真实电机话题。
- 远端本轮备份：`/home/run/LocoInfra_Q1_Dance/.codex_backups/20261113_1545_folded_route/`。

### 16:50 后：两段完整收储生命周期

- 原单段 `FOLDED_GETUP` 入口已拆为 `UNFOLD`（深折叠→平趴）和 `FOLD`（平趴→深折叠）。
- 启动自动链为 `UNFOLD→GETUP`；“折叠 / 准备关机”自动链为 `LIEDOWN→FOLD`。
- 普通“趴下”不会折叠；`FOLD` 完成后保持末帧，等 APP 关机，急停/断联仍切阻尼。
- 已用固件 crawl-to-fold 参考、实机深折叠终点和已有 `liedown_facedown` 构建两条 640 帧训练参考，并重算 MuJoCo 全身正运动学。
- 当前状态：数据、状态机、网页按钮、训练/导出脚本已补齐；两个 ONNX 未训练，不放行真机。

## 线路 B：人工标准平贴姿起身

- 新增独立仿真验收 `apps.teleop.prone_getup_sim_check`；验收只经过 `LYING → GETUP → WALK`，没有进入 `FOLDED / UNFOLD / FOLD`。
- MuJoCo 结果通过：最低站立 upright `0.9990`，最低站立骨盆高 `0.4203 m`，终态 upright `1.0000`，骨盆高 `0.4216 m`。
- `getup_test.py` 新增 `--check-only`：只读关节/IMU，检查是否为俯卧、是否深折叠、与起身首帧最大单关节差是否超过 `0.7 rad`，不发布电机指令。
- 真实起身前新增同样的标准平贴姿门禁和厂商高层运控并发门禁；演练模式保持只发假话题。
- Mac 已通过网线连接机器人（Mac `10.1.1.99`、Q1 `10.1.1.100`），APP 已切换「低层运控开发」，低层关节/IMU 状态话题正常。
- 人工平贴姿只读预检通过：重力方向约 `[1.00, -0.06, -0.03]`、`upright=+0.03`，最大首帧偏差 `0.47 rad < 0.70 rad`。
- 保持测试通过：最大跟踪偏差 `0.018 rad`，相对起点最大移动 `0.000 rad`，测试后自动切阻尼。
- 完整真机闭环已通过：9.1 s 起身、站立 5 s、8.5 s 趴下，站立阶段 `upright=+1.00`、角速度 `0.00–0.01 rad/s`。
- 策略段 1128 步、22.6 s，超拍 7 次；ONNX 推理 mean `11.4 ms`、p99 `17.0 ms`；关节状态共 4719 帧、命令共 12120 帧。
- 最终只读核验：机器人回到标准趴姿，重力方向约 `[1.00, -0.01, -0.01]`、`upright=+0.01`，与参考首帧最大差 `0.24 rad`；真实命令话题 Publisher 0、Subscriber 1，未发现控制器进程。
- 真机日志：`/home/run/LocoInfra_Q1_Dance/q1/getup/data/real_policy_20261113_182027.npz`；远端原脚本备份在 `.codex_backups/20261113_1808_line_b/`。

### 线路 B 独立网页（18:36）

- 新增 `apps.line_b_web`，默认端口 `8768`；线路 A 的 `apps.dance_web`（8767）和收储状态机未修改。
- 独立状态图只有 `LYING → GETUP → WALK ↔ DANCE → STRAIGHTEN → LIEDOWN → LYING`，代码中没有可执行的 `FOLDED / UNFOLD / FOLD` 状态。
- 网页和 API 均拒绝折叠/展开、横移和转向；只开放前后行走，前进硬上限 `0.08 m/s`。
- 标准平趴、非折叠和最大单关节首帧差 `≤0.7 rad` 三道门禁都在线路 B 独立状态机内部执行，门禁失败时不调用 `reset()`。
- Mac 端线路 B 10 项单元测试、线路 A/B 联合 18 项回归测试通过；机器人端线路 B 10 项单元测试通过。
- Mac 与机器人端完整 MuJoCo/ONNX 闭环均通过：两次进入 `WALK`，最终 `LYING`，最低行走 upright 约 `0.997`。
- 新目录已部署到 `/home/run/LocoInfra_Q1_Dance/apps/line_b_web/`；尚未启动真机网页控制，也未因此发布真实电机指令。

### 线路 B 网页首次真机事故与锁定（18:46–18:58）

- 首次启动独立网页后，人工点击起身；程序完成 `GETUP` 并自动进入 `WALK`，随后检测倾倒并切 `DAMPED`。
- 控制器已通过 `SIGTERM` 正常退出，8768 关闭，所有收发子进程消失，真实命令话题 Publisher 回到 0。
- 操作员确认机器人完全松软、无异响、无明显结构损伤；禁止再次启动原流程。
- 既有成功真机日志中，起身末帧保持阶段 upright 最低 `0.9982`、角速度最大 `0.014 rad/s`；离线重放得到基础行走策略首目标与起身目标最大差 `0.383 rad`，腿部差约 `0.108–0.137 rad`。失败运行没有逐帧日志，因此策略交接是首要嫌疑，但不是已证明的唯一根因。
- 线路 B 已事故锁定：`GETUP` 完成后进入 `STAND_HOLD` 并持续跟踪已验证的起身末帧；默认 API 拒绝所有非零移动与舞蹈，只允许再执行 `LIEDOWN`。
- 锁定后 Mac 端线路 A/B 联合 20 项测试、机器人端线路 B 12 项测试通过；两端安全仿真均只经过 `LYING → GETUP → STAND_HOLD → LIEDOWN → LYING`，保持阶段最低 upright `0.9994`。
- 锁定代码已同步机器人，但网页服务保持停止；在专项交接验收完成前不得恢复行走或舞蹈。

### 线路 B 起身到行走离线交接修复

- 本轮严格限定在 Mac 的 MuJoCo 离线环境，没有连接机器人、启动真机网页或发布电机指令。
- 交接候选已接入正式 `LineBMachine`，但配置缺省为关闭；只有显式 `--experimental-sim-handoff` 才会出现 `WALK_HANDOFF / WALK_SETTLE`。状态机与命令行双重拒绝非 MuJoCo 后端，真机和真机假话题演练都不能开启。
- 候选交接先用 `5.0 s` smoothstep 同时混合起身末帧/基础策略目标与两套 PD 增益，再以基础策略零速度观察 `3.0 s`；相邻关节目标变化硬限 `0.01 rad`。
- `0.4/1/2/3/5 s` 全流程扫描中，仅 `5 s` 同时满足未限幅原始目标变化 `≤0.01 rad/帧`；最大值为 `0.00725 rad`，最低直立度 `0.99880`，最大骨盆角速度 `0.03956 rad/s`，最大跟踪误差 `0.09170 rad`。
- 5 秒候选的 8 组鲁棒性回归全部通过：起身后保持 `0.5/2/5/10 s`，以及四组 `±0.01/±0.02 rad` 关节扰动。最差原始/实际单帧变化 `0.00729 rad`、最低直立度 `0.99876`、最大跟踪误差 `0.12231 rad`。
- 每组都完成 `GETUP → WALK_HANDOFF → WALK_SETTLE → WALK → DANCE → WALK → LIEDOWN → LYING`。
- 正式网页/API 的离线链路也完成同一流程；交接过程中模拟网页断联会产生 `DAMP` 并立即进入 `DAMPED`，默认不开实验开关时 API 拒绝 `walk`。
- Mac 本地网页人工验收完成：完整点击流程的按钮开放时机与状态提示正确；在第二轮 `WALK_HANDOFF` 中点击网页急停，页面与状态机立即进入 `DAMPED` 并锁住全部动作。浏览器控制台没有警告或错误，测试页与本地 8768 服务随后关闭。
- 线路 A/B、存储姿态和底层安全相关联合 45 项单元测试通过，正式状态机的 8 组鲁棒性复测仍为 `8/8`。
- 这仍只是离线候选；本轮没有连接、同步或启动机器人，真机仍运行上次部署的事故锁定代码，未解除行走/舞蹈锁定。

### 线路 B 连接机器人后的无电机验收（19:31–19:41）

- Mac 网线地址 `10.1.1.99`、Q1 `10.1.1.100` 正常，SSH 使用该机器人的专用密钥连接；机器人本次启动时间为 `19:28:39`。
- 未发现 `line_b_web`、`dance_web`、`primebot_io`、遥操作、起身测试或厂商 `mc_main / mc_app_main` 控制进程。
- `/aima/hal/joint/state` 与 `/aima/hal/imu/state` 各有 1 个 `device_manager_node` 发布者；真实 `/aima/hal/joint/command` Publisher 为 `0`、Subscriber 为 `1`。
- 机器人生产 `apps/line_b_web` 仍是事故锁定版本，没有 `WALK_HANDOFF`；测试前后五个生产文件 SHA256 完全一致。
- 候选只复制到 `/tmp/q1_line_b_handoff_validation.*` 隔离目录，并只读链接现有权重、参考和运行时；没有覆盖生产目录。21 项线路 B 单元测试通过。
- 机器人 CPU 的 `0.4/1/2/3/5 s` MuJoCo/ONNX 全流程扫描通过执行并仍选择 `5 s`：最大原始/实际单帧变化 `0.007242 rad`、最低直立度 `0.998791`、最大角速度 `0.040076 rad/s`、最大跟踪误差 `0.093907 rad`。
- 正式网页输入仲裁的无服务仿真通过：默认锁定、完整闭环、交接中断联 `DAMP → DAMPED` 全部符合预期。ONNX Runtime 仅提示未发现 GPU，随后正常使用 CPU。
- 3 秒既有假话题 dry-run 通过：策略 `150` 帧、`49.6 Hz`；关节状态 `598` 帧；假指令 `1469` 帧、`485.3 Hz`；推理 mean `9.783 ms`、p99 `21.72 ms`；`real_motor_topic_used=false`；当时 `upright=0.0328`。
- 结束后 `/kabutack/dry_run/joint/command` 已消失，真实命令 Publisher 仍为 `0`，没有控制或资源子进程。`/tmp` 隔离目录已删除，里面只有可重新生成的候选副本。

## 下一项（线路 A）

将 `q1/storage/motions/` 的两条参考放到有 NVIDIA GPU 的 SuperMimic-Q1 训练环境，训练共享策略并分别导出
`weights/q1_unfold` 和 `weights/q1_fold`。之后顺序是仿真分段验收、全链路仿真、真机假话题、吊带下单段低层验证，最后才允许网页调用。
