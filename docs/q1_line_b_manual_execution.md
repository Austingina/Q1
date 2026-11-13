# Q1 线路 B 手动执行手册

> 适用范围：标准平趴 → 起身 → 零速交接 → 人工行走/转向 ↔ 舞蹈 → 平趴。
> 线路 B 不包含 `FOLDED / UNFOLD / FOLD`；深度折叠姿态必须使用线路 A。

这份手册只整理已实现的手动命令与操作顺序。MuJoCo 通过不等于真机放行；
真机动作前仍必须现场确认吊带、独立急停、空旷地面、低层模式和标准平趴姿。

## 1. 快速选择

| 目的 | 后端 | 参数 | 真机动作 |
|---|---|---|---|
| 完整 MuJoCo 自动验收 | `sim` | `web_handoff_sim_check` | 否 |
| 本地网页手动仿真 | `sim` | `--enable-manual-drive --enable-dance` | 否 |
| 真机只读摆姿检查 | `real` | `--check-only` | 否 |
| 真机 CPU 推理门禁 | `real` | `--timing-check-only` | 否 |
| 真机假命令网页验证 | `real` | `--dry-run` | 否，只使用假话题 |
| 真机安全锁定网页 | `real` | 无 capability 参数 | 起身/趴下 |
| 真机人工行走 | `real` | `--enable-manual-drive` | 是 |
| 真机人工行走 + 舞蹈 | `real` | `--enable-manual-drive --enable-dance` | 是 |

## 2. Mac 离线环境

在 Mac 项目根目录执行：

```bash
cd "/Users/austin/Library/Containers/com.kingsoft.wpsoffice.mac/Data/Library/Application Support/Kingsoft/WPS Cloud Files/userdata/qing/filecache/482716163/gina计科学习/启元Q1/LocoInfra_Q1_Dance"

export PYTHONPATH="$PWD/infra/.line_b_tool_deps:$PWD${PYTHONPATH:+:$PYTHONPATH}"
export LINE_B_PYTHON=python3.13
```

### 2.1 语法与 49 项测试

```bash
"$LINE_B_PYTHON" -m py_compile \
  apps/line_b_web/__main__.py \
  apps/line_b_web/control.py \
  apps/line_b_web/machine.py \
  apps/line_b_web/web_handoff_sim_check.py \
  scripts/replace_line_b_dance.py

"$LINE_B_PYTHON" -m unittest \
  scripts.test_replace_line_b_dance \
  apps.line_b_web.test_control \
  apps.line_b_web.test_machine \
  apps.line_b_web.test_offline_handoff \
  apps.line_b_web.test_real_handoff_validation
```

### 2.2 完整 MuJoCo 自动验收

```bash
"$LINE_B_PYTHON" -m apps.line_b_web.web_handoff_sim_check
```

成功时末尾应显示：

```text
"full_web_flow": { ... "passed": true },
"disconnect_during_handoff": { ... "passed": true },
"passed": true
```

流程应包含：

```text
LYING → GETUP → STAND_HOLD → WALK_HANDOFF → WALK_SETTLE
      → WALK → STRAIGHTEN → DANCE → WALK
      → STRAIGHTEN → LIEDOWN → LYING
```

### 2.3 本地网页手动仿真

```bash
"$LINE_B_PYTHON" -m apps.line_b_web \
  --backend sim \
  --enable-manual-drive \
  --enable-dance \
  --speed-scale 0.5 \
  --host 127.0.0.1 \
  --port 18768
```

终端会打印带随机令牌的地址：

```text
http://127.0.0.1:18768/?token=<令牌>
```

仿真结束时在服务终端按 `Ctrl-C`。

## 3. 替换舞蹈包

不能只替换 `motion.npz`。新舞蹈必须是同次训练导出的完整包，包含：

```text
policy.onnx
deploy.yaml
motion.npz
deploy_motion.npz
meta.json
```

双击项目根目录中的 `replace_line_b_dance.command`，或在终端执行：

```bash
./replace_line_b_dance.command /path/to/new_dance_package
```

只检查，不替换：

```bash
./replace_line_b_dance.command --check-only /path/to/new_dance_package
```

成功替换会自动备份旧包、原子切换并运行完整 MuJoCo 验收；仿真失败会自动恢复旧包。
此命令只修改 Mac 候选，不会同步真机。

## 4. 真机动作前硬条件

以下条件必须同时满足：

- 机器人不在充电；
- 已在 APP 选择「低层运控开发」，厂商 `mc` 未运行；
- 机器人由吊带或可靠的软支撑保护；
- 独立硬件急停可立即使用；
- 周围无人、无障碍物、地面防滑；
- 结构无损伤、无异响，各关节在合理范围；
- 机器人已人工摆到标准平趴，不是深度折叠姿态；
- 真机代码和舞蹈包已另行同步并核对，服务启动前没有其他控制进程。

> 机器人正在充电时，只执行第 2–3 节的 Mac 离线命令，不执行以下真机命令。

## 5. 机器人终端环境

以下命令在机器人本机终端或连入机器人的 SSH 终端中执行：

```bash
cd /home/run/LocoInfra_Q1_Dance

source /opt/ros/humble/setup.bash
source /robot/software/interface/setup.bash
export PYTHONPATH=.python${PYTHONPATH:+:$PYTHONPATH}
```

检查当前参数入口：

```bash
python3 -m apps.line_b_web --help
```

## 6. 真机无动作检查

### 6.1 标准平趴只读检查

```bash
python3 -m apps.line_b_web.real_handoff_validation \
  --backend real \
  --check-only
```

它只读取 IMU 和关节，不调用电机 `reset()` / `step()`。结果必须是非折叠标准俯卧并显示通过。

### 6.2 CPU 推理时延门禁

```bash
python3 -m apps.line_b_web.real_handoff_validation \
  --backend real \
  --timing-check-only
```

这一步只做策略预热和 CPU 推理，不执行动作。平均推理不得超过 `15 ms`，
p99 不得超过 `18 ms`。

### 6.3 假命令话题网页验证

```bash
python3 -m apps.line_b_web \
  --backend real \
  --enable-real \
  --dry-run \
  --enable-manual-drive \
  --enable-dance \
  --speed-scale 0.5 \
  --host 127.0.0.1 \
  --port 18768
```

此模式只使用 `/kabutack/dry_run/joint/command`，不应对机器人产生动作。检查完后按 `Ctrl-C` 退出。

## 7. 真机网页启动命令

三种入口只选一种，不同时启动多个线路 B 进程。

### 7.1 安全锁定：只起身/趴下

```bash
python3 -m apps.line_b_web \
  --backend real \
  --enable-real \
  --speed-scale 0.5 \
  --host 0.0.0.0 \
  --port 8768
```

状态图：

```text
LYING → GETUP → STAND_HOLD → LIEDOWN → LYING
```

### 7.2 人工行走/转向，不启用舞蹈

```bash
python3 -m apps.line_b_web \
  --backend real \
  --enable-real \
  --enable-manual-drive \
  --speed-scale 0.5 \
  --host 0.0.0.0 \
  --port 8768
```

### 7.3 人工行走/转向 + 完整舞蹈

```bash
python3 -m apps.line_b_web \
  --backend real \
  --enable-real \
  --enable-manual-drive \
  --enable-dance \
  --speed-scale 0.5 \
  --host 0.0.0.0 \
  --port 8768
```

当前原仓库参数映射：

```text
speed-scale = 0.5
前进上限    = +0.30 m/s
后退上限    = -0.20 m/s
左/右转上限 =  0.50 rad/s
线速度斜坡    =  1.0 m/s²
转向斜坡      =  3.0 rad/s²
控制频率      = 50 Hz
```

`--speed-scale` 最大只允许 `0.5`；不要将它改成仿真满速 `1.0`。

## 8. 打开网页

启动后终端会打印：

```text
[line-b] 打开 http://<机器人IP>:8768/?token=<随机令牌>
```

根据当前网络选择手机或 Mac 可达的机器人 IP：

```text
http://<机器人当前IP>:8768/?token=<终端打印的令牌>
```

历史网线直连配置中，机器人曾使用 `10.1.1.100`；只有现场网络仍为该配置时才使用：

```text
http://10.1.1.100:8768/?token=<令牌>
```

手机必须能够路由到这个 IP；仅“连上与 Mac 同名的 Wi-Fi”并不必然代表可达。

## 9. 网页手动操作顺序

1. 确认网页显示线路 `B`、状态 `LYING`、网页连接正常。
2. 点击“一键检查摆姿”；只有显示“检查通过”才继续。
3. 点击“起身”，等待 `GETUP → STAND_HOLD`；不要在中途拖拽机器人。
4. 如启用人工行走，点击“进入人工行走”，等待：

   ```text
   WALK_HANDOFF（10 s）→ WALK_SETTLE（3 s）→ WALK
   ```

   交接期间不按任何方向按钮。

5. 只在页面显示 `WALK` 后使用六个方向按钮：

   ```text
   按住前进    按住后退
   按住左转    按住右转
   按住前进左转  按住前进右转
   ```

   每次只按一个，松手后确认 `vx=0.00` 且 `wz=0.00` 再做下一个动作。

6. 如启用舞蹈，只在 `WALK` 且现场稳定时点击“跳舞（完整动作）”：

   ```text
   WALK → STRAIGHTEN（速度归零、站稳）→ DANCE → WALK
   ```

   默认舞蹈约 `20.3 s`。`DANCE` 期间不执行行走、转向或趴下命令。

7. 正常结束时，先松开所有方向并确认 `vx/wz=0.00`，再点击“趴下”：

   ```text
   WALK → STRAIGHTEN → LIEDOWN → LYING
   ```

8. 等待页面显示 `LYING`、机器人已贴地并切阻尼后，才在机器人终端按 `Ctrl-C` 停止服务。

## 10. 断联、急停与退出语义

| 事件 | 行为 |
|---|---|
| 松开方向按钮 | 速度按原仓库斜坡归零 |
| `0.25 s` 没有新移动命令 | 人工速度归零 |
| 网页失焦/关闭/断网/心跳超过 `1.0 s` | `vx/wz` 归零，保持当前控制状态，不自动切阻尼 |
| 点击红色“立即阻尼急停” | 显式进入 `DAMPED` |
| 状态机检测到倾倒 | 自动进入 `DAMPED` |
| 真机服务终端 `Ctrl-C`/进程退出 | 后端会切阻尼后停止收发进程 |

网页断联不会自动阻尼，但也不是硬件急停。独立急停必须一直在操作者手边。

## 11. 异常后处置

如果进入 `DAMPED`、跌倒或机器人被吊带接住：

1. 停止发送所有方向命令，使用独立急停保证无扭矩。
2. 检查是否完全松软、有无异响和明显结构损伤。
3. 人工重新摆到标准平趴；不从坐姿、半跪、折叠或吊带悬挂姿态直接再次起身。
4. 重新执行“一键检查摆姿”；不通过就停止。

## 12. 参数约束

```text
--backend real          选择真机后端
--enable-real           真机二次确认，缺少就拒绝启动
--enable-manual-drive   显式开放零速交接和人工行走/转向
--enable-dance          显式开放舞蹈，必须与 --enable-manual-drive 同用
--speed-scale 0.5       原仓库真机比例，代码禁止超过 0.5
--drive-timeout 0.25    人工移动命令超时
--heartbeat-timeout 1.0 网页心跳超时，只归零并保持状态
--host 0.0.0.0          允许可达机器人 IP 的 Mac/手机访问
--port 8768             线路 B 独立端口；线路 A 默认为 8767
```

## 13. 不得执行的组合

```text
--backend real                  但缺少 --enable-real
--enable-dance                 但缺少 --enable-manual-drive
--experimental-sim-handoff     与 --backend real 同用
--experimental-sim-handoff     与 --enable-manual-drive 同用
--speed-scale > 0.5
```

这些组合都会被代码拒绝，不要修改源码绕过。

## 14. 文件与入口

```text
apps/line_b_web/__main__.py                 线路 B 网页入口
apps/line_b_web/machine.py                  状态机
apps/line_b_web/control.py                  网页输入仲裁/断联策略
apps/line_b_web/index.html                  手机网页
apps/line_b_web/real_handoff_validation.py  一次性零速交接验收
apps/line_b_web/web_handoff_sim_check.py    完整 MuJoCo 网页链路验收
replace_line_b_dance.command                完整舞蹈包一键替换
scripts/replace_line_b_dance.py             替换、备份、回滚与仿真验收实现
weights/q1_smooth_dance/                    当前线路 B 舞蹈包
```

## 15. 最短真机执行清单

```text
1. 不充电，低层模式，吊带/急停/场地就位
2. 人工标准平趴
3. --check-only 通过
4. --timing-check-only 通过
5. 只启动一个线路 B 真机网页进程
6. 网页一键检查摆姿通过
7. 起身 → STAND_HOLD
8. 进入人工行走 → 等待 WALK
9. 人工行走/转向或舞蹈
10. 归零 → 趴下 → LYING
11. 在 LYING 后 Ctrl-C 停服务
```
