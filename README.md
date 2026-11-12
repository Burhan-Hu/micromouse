# Micro-mouse

基于 **TQD-Micromouse-OC v2.0** 与 ROS 2 Humble / Gazebo 的电脑鼠迷宫竞速项目。仓库包含 TQD 仿真模型、竞赛控制程序、融合寻路算法、离线评测工具，以及固定全迷宫视角的实时观察窗。

> GitHub 仓库：[Burhan-Hu/micromouse](https://github.com/Burhan-Hu/micromouse)（Private）

## 项目功能

- **融合探索与竞速算法**：结合洪水填充和探索偏置进行迷宫探索；确认起点至中心目标区的最短路径所需信息后，执行路径校验和最短路冲刺。
- **ROS 2 / Gazebo 闭环控制**：竞赛程序通过 `/scan`、`/odom` 获取传感器与里程计信息，并通过 `/cmd_vel` 控制机器人。
- **离线开发与评测**：提供 Bulebule 风格基线、mms 风格算法实现、融合算法以及比较、噪声和定位漂移评测工具。
- **可视化观察**：支持 Gazebo 3D 窗口和固定全迷宫视野的实时观察窗，显示机器人位置与运行轨迹。
- **仿真地图校验**：脚手架可从 `.TQD` 和 Gazebo `.world` 解析迷宫，用于检查 maze1–3 的墙体布局。

算法实现用于本项目接口与仿真环境，不是对参考项目源代码的直接复制。`tools/README.md` 记录了算法来源与开发脚手架细节。

## 环境要求

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Classic 及 `gazebo_ros`（包含 `gzserver`、`gzclient`）
- Python 3.10（与 ROS Humble 的 `rclpy` 兼容）

本机若默认 `python3` 指向 Anaconda Python 3.12，运行 ROS 脚本前请切换系统 PATH。项目启动脚本也会设置系统 PATH。

## 快速开始

### 1. 启动 Gazebo 图形仿真

在有桌面显示的终端中，从仓库根目录运行：

```bash
tools/sim_gui.sh
```

默认加载 maze1，启动 Gazebo 物理仿真、生成机器人并打开 3D 窗口。也可以单独启动无 GUI 的仿真：

```bash
tools/sim_up.sh
```

指定其他迷宫：

```bash
tools/sim_gui.sh "TQD-Micromouse-OC v2.0/world/maze2.world"
```

### 2. 运行竞赛程序

仿真启动后，在**另一个终端**运行：

```bash
cd "TQD-Micromouse-OC v2.0"
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
source /opt/ros/humble/setup.bash
python3 contest.py
```

Gazebo 窗口仅负责显示，不会自动驱动机器人；必须运行竞赛程序才会向 `/cmd_vel` 发布控制指令。程序到达中心目标区后会报告完成并停止发布速度，此时机器人静止是正常现象。

### 3. 打开固定全迷宫观察窗（可选）

在仓库根目录另开终端：

```bash
source /opt/ros/humble/setup.bash
python3 tools/maze_view.py
```

该窗口固定显示整个迷宫，不会跟随机器人缩放。常用选项：

```bash
python3 tools/maze_view.py --size 1100          # 调整窗口尺寸
python3 tools/maze_view.py --record run.mp4      # 同时录制
python3 tools/maze_view.py --no-trail            # 隐藏轨迹
```

### 4. 停止仿真

```bash
tools/sim_down.sh
```

## 离线评测

无需启动 Gazebo，可从仓库根目录运行：

```bash
python3 tools/offline_eval.py                 # maze1–3 指标
python3 tools/offline_eval.py --noise 0.02    # 加入 2% 传感器误判
python3 tools/compare_eval.py                 # 对比基线与算法
python3 tools/robust_eval.py                  # 定位漂移鲁棒性评测
```

在线脚手架、算法模块说明、迷宫坐标约定及测距阈值见 [`tools/README.md`](tools/README.md)。

## 目录结构

```text
.
├── TQD-Micromouse-OC v2.0/   # TQD 仿真模型、迷宫地图和竞赛程序
│   ├── contest.py            # ROS 2 竞赛控制程序
│   ├── TQD/                  # maze1–3 的 TQD 地图
│   ├── urdf/                 # 机器人模型
│   └── world/                # Gazebo 世界文件
├── tools/                    # 算法、离线评测、仿真脚本和观察窗
├── TQD/                      # 迷宫地图及资源
├── urdf/                     # 机器人 URDF
└── world/                    # Gazebo 世界文件
```

## 目标与约定

- 迷宫为 16×16 格，单格边长 0.18 m。
- 起点为 `(0, 0)`；目标为迷宫中央的四格区域：`(7,7)`、`(7,8)`、`(8,7)`、`(8,8)`。
- 迷宫布局可通过 `tools/maze_model.py` 从 `.TQD` 或 `.world` 文件读取。

## 注意事项

- 仿真启动脚本会生成临时 world 副本并将迷宫设为 static，避免机器人碰撞造成迷宫整体移动。
- `gzclient` 图形窗口异常退出不必然表示 `gzserver` 物理仿真停止；可检查 `/tmp/gzserver.log` 和 ROS 话题。
- 速度规划实验结果不等同于比赛成绩；竞赛程序采用的是仿真中验证过的保守控制速度。
