# tools/ — 开发脚手架

面向 TQD-OC V2.0 仿真评测系统的本地开发闭环：离线跑算法、在线跑 Gazebo、量化指标。

## 环境前提（重要）

`PATH` 里的 anaconda `python3` 是 3.12，而 ROS Humble 的 `rclpy` 只为 **3.10** 编译。
所有脚本已把 `PATH` 重置为系统路径，自己跑命令时也要注意：

```bash
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
source /opt/ros/humble/setup.bash
```

## 仿真控制

```bash
tools/sim_up.sh [world] [x] [y] [yaw]   # 起 gzserver + 生成机器人（默认 maze1，单元格(0,0)中心，朝北）
tools/sim_reset.sh                      # 机器人复位到起点
tools/sim_down.sh                       # 停止 gzserver
tools/sim_probe.py [out.png]            # 自检：打印 5 通道测距 + 里程计 + 俯视截图
```

`sim_up.sh` 会先用 `prepare_world.py` 生成 `/tmp/mm_*.world`，把 `my_maze` 设为
`static`。**原厂 world 里迷宫不是 static，被撞会整体漂移**，导致真值与传感器失配。

## 离线算法开发（秒级，无需 Gazebo）

```bash
python3 tools/offline_eval.py                     # maze1..3 全指标
python3 tools/offline_eval.py --noise 0.02        # 2% 传感器误判
python3 tools/offline_eval.py --verbose           # 打印探索过程
python3 tools/compare_eval.py                    # Bulebule vs mms 对比
python3 tools/compare_eval.py --noise 0.02       # 噪声下对比
python3 tools/live_run.py --algo fused --maze 1  # 在线跑融合算法
```

## 在线算法验证（Gazebo 闭环）

```bash
tools/sim_up.sh
python3 tools/live_run.py --maze 1 --speed-x 0.3 --speed-z 1.2
python3 tools/live_run.py --algo mms --maze 1 --speed-x 0.3 --speed-z 1.2
```

## 模块

| 文件 | 作用 |
|---|---|
| `maze_model.py` | 16×16 迷宫模型；`from_world()` 从 Gazebo 几何反推墙壁，`from_tqd()` 解析 .TQD。两者对 maze1/2/3 **逐墙零误差**一致 |
| `baseline_floodfill.py` | Bulebule 风格洪水填充基线（独立实现，未复制 GPLv3 源码） |
| `baseline_mms.py` | mms 风格洪水（移植自 ShifatHasanGNS/maze_solver, MIT）：探索偏置 + 首次到达即停 + 路径压缩 |
| `fused_search.py` | **融合算法**：mms 探索偏置 + 路径全已知才收工 + 冲刺前校验回合 |
| `compare_eval.py` | 多套算法对比评测 |
| `robust_eval.py` | 定位漂移（打滑/漏计）鲁棒性评测 |
| `robot.py` | `SimRobot`（离线理想机器人）/ `RosRobot`（Gazebo 闭环驱动） |
| `offline_eval.py` | 离线指标 |
| `live_run.py` | 在线指标 + 建图诊断（`/tmp/live_known.txt`） |
| `prepare_world.py` | 生成 static 迷宫副本 |

## 坐标与阈值约定

- 迷宫坐标 `(i, j)`，i 沿 +x、j 沿 +y；单元格 0.18 m，中心 `-1.35 + 0.18*i`
- 起点单元格 (0,0) 中心 `(-1.35, -1.35)`，车头朝北（`yaw + π = 4.71`）
- 朝向常量：北 4.71 / 东 3.14 / 南 1.57 / 西 0(≈6.28)
- 单点测距（模板里的 340/270/180/90/20）在车体偏转时极不稳定，`RosRobot` 改为
  **角度区间取第 3 小值**（规避 Gazebo 偶发的单条射线野值）；建图阈值用 0.15
  （侧墙实测 ≈0.084，无墙 ≥0.17）

## .TQD 格式（已验证）

每行 33 字符，共 16 行，第 0 行为最北一行：
- 偶数列 `'|'` = 该格 **西墙**（最后一列是第 15 格的东墙）
- 奇数列 `'_'` = 该格 **南墙**
