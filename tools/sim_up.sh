#!/usr/bin/env bash
# 启动仿真：gzserver(带 ROS 插件) + 生成机器人
# 用法: tools/sim_up.sh [world文件] [起点x] [起点y] [偏航rad]
#
# 默认迷宫 world/maze1.world，起点为单元格 (0,0) 中心 (-1.35,-1.35)，
# 偏航 +pi/2 使 yaw+pi = 4.71（与 config.ini 的 fyaw 一致，车头朝北）。
set -o pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
APP="$ROOT/TQD-Micromouse-OC v2.0"
WORLD="${1:-$APP/world/maze1.world}"
PX="${2:--1.35}"
PY="${3:--1.35}"
PYAW="${4:-1.5708}"
PIDFILE="/tmp/micromouse_gzserver.pid"

# 关键：anaconda 的 python3(3.12) 会让 rclpy 失效，这里强制使用系统 python3.10
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash >/dev/null 2>&1 || true

if pgrep -f "gzserver" >/dev/null 2>&1; then
  echo "已有 gzserver 在运行，先执行 tools/sim_down.sh"
  exit 1
fi

# 原厂 world 的 my_maze 不是 static，会被机器人撞跑 -> 生成 static 副本
WORLD_RUN="/tmp/mm_$(basename "${WORLD%.world}").world"
python3 "$ROOT/tools/prepare_world.py" "$WORLD" "$WORLD_RUN" || exit 2

echo "[1/3] 启动 gzserver: $WORLD_RUN"
nohup gzserver -s libgazebo_ros_init.so -s libgazebo_ros_factory.so "$WORLD_RUN" \
  >/tmp/gzserver.log 2>&1 &
echo $! > "$PIDFILE"

echo "[2/3] 等待 /spawn_entity 服务..."
for _ in $(seq 1 60); do
  if ros2 service list 2>/dev/null | grep -q '/spawn_entity'; then break; fi
  sleep 1
done

echo "[3/3] 生成机器人 @ ($PX, $PY, yaw=$PYAW)"
ros2 run gazebo_ros spawn_entity.py \
  -file "$APP/urdf/robot_4wheel_final.urdf" \
  -entity robot -x "$PX" -y "$PY" -z 0.01 -Y "$PYAW" 2>&1 | tail -2

sleep 1
echo "就绪。话题："
ros2 topic list | grep -E '^/(scan|odom|cmd_vel)$'
echo "日志: /tmp/gzserver.log    停止: tools/sim_down.sh"
