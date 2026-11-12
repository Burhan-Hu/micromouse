#!/usr/bin/env bash
# 停止仿真
set -o pipefail
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
source /opt/ros/humble/setup.bash >/dev/null 2>&1 || true
PIDFILE=/tmp/micromouse_gzserver.pid

if [ -f "$PIDFILE" ]; then
  pid=$(cat "$PIDFILE")
  if kill -0 "$pid" 2>/dev/null; then
    echo "停止 gzserver pid=$pid"
    kill -INT "$pid"
    for _ in $(seq 1 20); do kill -0 "$pid" 2>/dev/null || break; sleep 0.5; done
    kill -0 "$pid" 2>/dev/null && kill -9 "$pid"
  fi
  rm -f "$PIDFILE"
fi
pgrep -a gzserver || echo "gzserver 已全部退出"
