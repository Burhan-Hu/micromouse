#!/usr/bin/env bash
# 启动"带图形界面"的仿真：gzserver + gzclient + 机器人
# 用法: tools/sim_gui.sh [world文件]
#
# 说明：
#   gzserver 负责物理与传感器（必须），gzclient 只是 3D 观察窗口，
#   它崩了不影响仿真与程序运行，重启即可。
set -o pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
APP="$ROOT/TQD-Micromouse-OC v2.0"
WORLD="${1:-$APP/world/maze1.world}"
PIDFILE="/tmp/micromouse_gzserver.pid"
CLIENT_PID="/tmp/micromouse_gzclient.pid"

export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
source /opt/ros/humble/setup.bash >/dev/null 2>&1 || true

if [ -z "${DISPLAY:-}" ]; then
  echo "没有 DISPLAY：请在有桌面的终端里运行，或先 export DISPLAY=:0" >&2
  exit 1
fi

# 原厂 world 的 my_maze 不是 static，被撞会整体漂移 -> 生成 static 副本
WORLD_RUN="/tmp/mm_$(basename "${WORLD%.world}").world"
python3 "$ROOT/tools/prepare_world.py" "$WORLD" "$WORLD_RUN" >/dev/null || exit 2

if pgrep -f gzserver >/dev/null 2>&1; then
  echo "已有 gzserver 在跑，先执行 tools/sim_down.sh"
  exit 1
fi

echo "[1/4] gzserver 启动（物理 + 传感器）"
nohup gzserver -s libgazebo_ros_init.so -s libgazebo_ros_factory.so "$WORLD_RUN" \
  >/tmp/gzserver.log 2>&1 &
echo $! > "$PIDFILE"

echo "[2/4] 等待 /spawn_entity"
for _ in $(seq 1 60); do
  ros2 service list 2>/dev/null | grep -q '/spawn_entity' && break
  sleep 1
done

echo "[3/4] 生成机器人 @ 单元格(0,0) 中心，朝北"
ros2 run gazebo_ros spawn_entity.py \
  -file "$APP/urdf/robot_4wheel_final.urdf" \
  -entity robot -x -1.35 -y -1.35 -z 0.01 -Y 1.5708 2>&1 | tail -1

echo "[4/4] gzclient 打开 3D 观察窗口"
nohup gzclient --gui-client-plugin=libgazebo_ros_eol_gui.so \
  >/tmp/gzclient.log 2>&1 &
echo $! > "$CLIENT_PID"
sleep 3

if kill -0 "$(cat "$CLIENT_PID")" 2>/dev/null; then
  echo "✅ 窗口已打开（拖动鼠标左键旋转，滚轮缩放，Shift+拖动平移）"
else
  echo "⚠️ gzclient 退出，看 /tmp/gzclient.log；仿真本身仍在运行" >&2
fi

cat <<'TIP'

★ 为什么现在车是静止的？
    gzclient 只是"观察窗口"，它不驱动机器人。必须有人往 /cmd_vel 发速度指令。
    所以：**另开一个终端跑算法**，车才会动。

    cd "TQD-Micromouse-OC v2.0"
    export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
    source /opt/ros/humble/setup.bash
    python3 contest.py

★ 看不清车？车体只有 9cm×6cm，迷宫却有 2.9m：
    · 鼠标滚轮放大（缩放到约 0.5m 视野）
    · 在左侧 World 面板里点 robot，右键 -> Follow，视角就会跟着车
    · 菜单 View -> Wireframe 关掉可以让画面更清爽

★ 想确认物理在跑：看窗口底部 Sim Time 是否在涨（正常 RTF 约 4，比实时快 4 倍）
TIP
