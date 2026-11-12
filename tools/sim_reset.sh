#!/usr/bin/env bash
# 把机器人复位到起点单元格 (0,0) 中心，车头朝北。
# 优先用 gz model 直接改位姿（不删不建，factory 服务更稳）；
# 失败则回退到 删除+重建。
set -o pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
APP="$ROOT/TQD-Micromouse-OC v2.0"
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
source /opt/ros/humble/setup.bash >/dev/null 2>&1 || true

X=-1.35; Y=-1.35; Z=0.01; YAW=1.5708

if timeout 8 gz model -m robot -x "$X" -y "$Y" -z "$Z" -R 0 -P 0 -Y "$YAW" >/dev/null 2>&1; then
  if timeout 8 gz model -m robot -p >/dev/null 2>&1; then sleep 1; echo "复位完成(改位姿)"; exit 0; fi
fi

echo "改位姿失败，回退到重建..." >&2
timeout 6 gz model -m robot -d >/dev/null 2>&1
sleep 2
for attempt in 1 2 3; do
  out=$(ros2 run gazebo_ros spawn_entity.py \
        -file "$APP/urdf/robot_4wheel_final.urdf" \
        -entity robot -x "$X" -y "$Y" -z "$Z" -Y "$YAW" 2>&1 | grep -E "Successfully|failed|exists" | tail -1)
  case "$out" in *Successfully*) sleep 1; echo "复位完成(重建)"; exit 0;; esac
  sleep 2
done
echo "复位失败: $out" >&2
exit 1
