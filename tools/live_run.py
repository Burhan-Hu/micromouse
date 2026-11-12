"""在 Gazebo 仿真中跑 Bulebule 风格基线，实测关键指标。

前置：先 tools/sim_up.sh
用法：
    python3 tools/live_run.py --maze 1 --speed-x 0.35 --speed-z 1.2
"""
import argparse
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from maze_model import N, E, S, W, DIRS, from_world, from_tqd
from robot import RosRobot, rel_to_abs
from baseline_floodfill import FloodFillSearch, FRONT, LEFT, RIGHT, BACK, turn_left, turn_right
from baseline_mms import MMSSearch
from fused_search import FusedSearch

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
APP = os.path.join(ROOT, "TQD-Micromouse-OC v2.0")
CW = [N, E, S, W]


def rel_between(heading, target_dir):
    delta = (CW.index(target_dir) - CW.index(heading)) % 4
    return {0: FRONT, 1: RIGHT, 2: BACK, 3: LEFT}[delta]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--maze", default="1")
    ap.add_argument("--speed-x", type=float, default=0.35)
    ap.add_argument("--speed-z", type=float, default=1.2)
    ap.add_argument("--max-moves", type=int, default=1200)
    ap.add_argument("--no-run", action="store_true", help="只做探索，不冲刺")
    ap.add_argument("--config", default=os.path.join(APP, "config.ini"))
    ap.add_argument("--algo", choices=["bulebule", "mms", "fused"], default="bulebule")
    args = ap.parse_args()

    import rclpy
    rclpy.init(args=None)

    robot = RosRobot(args.config, speed_x=args.speed_x, speed_z=args.speed_z)
    ok = robot.start()
    print(f"[连接] odom {'就绪' if ok else '超时'}  pos=({robot.x:.3f},{robot.y:.3f}) yaw+pi={robot.yaw:.3f}")
    if not ok:
        print("没有收到 /odom，请先 tools/sim_up.sh")
        return 1
    # 清理上一轮残留速度，并确保车头朝北（gz model 改位姿不会清角速度）
    robot.stop()
    for _ in range(15):
        robot.rate.sleep()
    if abs(robot._wrap(math.pi / 2 - robot.world_yaw())) > 0.12:
        print("[连接] 车头未对正，先摆正到北")
        robot.turn_to_dir(N)

    # 进度计数 + 步数上限保护
    state = {"n": 0, "t0": time.time()}

    class CountingRobot:
        def __init__(self, inner):
            self.inner = inner

        def read_walls(self):
            w = self.inner.read_walls()
            print(f"  [感知] pos={pos[0]} odom=({self.inner.x:+.3f},{self.inner.y:+.3f}) "
                  f"yaw+pi={self.inner.yaw:.3f} l={self.inner.l_dis:.3f} "
                  f"f={self.inner.f_dis:.3f} r={self.inner.r_dis:.3f} -> {w}", flush=True)
            return w

        def move_step(self, rel, heading):
            state["n"] += 1
            if state["n"] % 20 == 0:
                print(f"  [进度] 第 {state['n']} 步  pos={pos[0]}  "
                      f"耗时 {time.time()-state['t0']:.0f}s  "
                      f"odom=({self.inner.x:+.2f},{self.inner.y:+.2f})", flush=True)
            if state["n"] > args.max_moves:
                raise RuntimeError("超过最大步数限制")
            self.inner.move_step(rel, heading)

    # 起点/朝向与 world 一致：单元格(0,0) 中心，朝北
    if args.algo == "mms":
        algo = MMSSearch(robot, start=(0, 0), heading=N, goals=GOALS_LIVE, verbose=True)
    elif args.algo == "fused":
        algo = FusedSearch(robot, start=(0, 0), heading=N, goals=GOALS_LIVE, verbose=True)
    else:
        algo = FloodFillSearch(robot, start=(0, 0), heading=N, verbose=True)
    pos = [algo.pos]
    _orig_apply = algo._apply_step
    algo._apply_step = lambda rel: (_orig_apply(rel), pos.__setitem__(0, algo.pos))
    algo.robot = CountingRobot(robot)

    # 用里程计回算真实格子与朝向，替代纯航迹推算（防止累积漂移）
    def _apply_step_odom(rel):
        i, j = robot.cell_from_odom()
        algo.pos = (i, j)
        algo.heading = robot.heading_from_yaw()
        pos[0] = (i, j)
    algo._apply_step = _apply_step_odom

    print("\n===== 探索阶段 =====")
    t0 = time.time()
    algo.explore()
    if args.algo == "fused":
        print("\n===== 校验阶段 =====")
        algo.verify()
    t_exp = time.time() - t0
    visited = getattr(algo, "visited", None) or getattr(algo, "explored", set())
    print(f"探索结束：{algo.stats.moves} 格 / {algo.stats.turn90}×90° / "
          f"{algo.stats.turn180}×180° / 感知 {algo.stats.sense} 次 / "
          f"访问 {len(visited)} 格  用时 {t_exp:.1f}s")

    # ---- 诊断：导出已知迷宫，与真值对比 ----
    truth = from_world(os.path.join(ROOT, "world", f"maze{args.maze}.world"))
    known = getattr(algo, "known", None) or getattr(algo, "walls", None)
    with open("/tmp/live_known.txt", "w") as f:
        f.write("== 已知迷宫 ==\n" + ascii_of(known) + "\n\n== 真值 ==\n" + truth.to_ascii() + "\n")
    bad = [(i, j, known[i][j], truth.walls[i][j]) for (i, j) in visited
           if known[i][j] != truth.walls[i][j]]
    print(f"[诊断] 已访问格中墙壁错误 {len(bad)} 个 -> /tmp/live_known.txt")
    for b in bad[:12]:
        print("   cell(%d,%d) known=%02x truth=%02x" % b)

    if args.algo == "mms":
        seq = algo.run_sequence()
        run_ops = []
        hd = N
        for ch in seq:
            if ch == "L":
                hd = turn_left(hd)
                run_ops.append(("T", hd))
            elif ch == "R":
                hd = turn_right(hd)
                run_ops.append(("T", hd))
            elif ch == "F":
                run_ops.append(("F", hd))
        run_path = run_ops
    else:
        run_path = algo.build_run_path()
    run_cells = (sum(1 for it in run_path if it[0] == "F") if args.algo == "mms"
                 else len(run_path))
    print(f"冲刺路线长度: {run_cells} 格")

    t_run = 0.0
    if not args.no_run and run_path:
        print("\n===== 冲刺阶段 =====")
        # 回到起点并摆正车头（用已知地图洪水导航回去，只依赖已探测的墙）
        drive_home(robot, known)
        for _ in range(10):
            robot.rate.sleep()
        if abs(robot._wrap(math.pi / 2 - robot.world_yaw())) > 0.10:
            robot.turn_to_dir(N)
        t0 = time.time()
        for k, item in enumerate(run_path):
            if (k + 1) % 10 == 0:
                print(f"  {k+1}/{len(run_path)} op  odom=({robot.x:.2f},{robot.y:.2f}) f={robot.f_dis:.3f}", flush=True)
            if args.algo == "mms":
                kind, d = item
                if kind == "T":
                    robot.turn_to_dir(d)
                    continue
                robot.move_step(FRONT, d)
                continue
            ni, nj, d = item
            heading = robot.heading_from_yaw()   # 每步都用里程计重新同步，避免误差累积
            rel = rel_between(heading, d)
            robot.move_step(rel, heading)
        t_run = time.time() - t0
        print(f"冲刺完成，用时 {t_run:.1f}s")

    # 与真值对比
    truth = from_world(os.path.join(ROOT, "world", f"maze{args.maze}.world"))
    optimal = truth.bfs_path_len((0, 0), [(7, 7), (7, 8), (8, 7), (8, 8)])
    err = sum(1 for (i, j) in visited if known[i][j] != truth.walls[i][j])

    print("\n===== 关键指标 =====")
    print(f"理论最短路径        : {optimal} 格")
    print(f"冲刺路径长度        : {run_cells} 格  (效率 {optimal / max(run_cells,1):.1%})")
    print(f"探索移动/转向       : {algo.stats.moves} 格 / {algo.stats.turn90}×90° / {algo.stats.turn180}×180°")
    print(f"探索耗时(实测)      : {t_exp:.1f} s")
    print(f"冲刺耗时(实测)      : {t_run:.1f} s")
    print(f"已访问区墙壁记忆错误: {err}")
    if t_run:
        score = t_exp / 30.0 + t_run - 2.0
        print(f"排障时间(迷宫时间/30+运行时间-2s): {score:.1f} s")

    robot.stop()
    robot.close()
    sys.stdout.flush()
    os._exit(0)


GOALS_LIVE = [(7, 7), (7, 8), (8, 7), (8, 8)]


def drive_home(robot, known, size=16, target=(0, 0), max_ops=400):
    """用已知墙壁做 BFS，从当前格走回起点（只用已探测信息）。"""
    from collections import deque
    dist = {target: 0}
    q = deque([target])
    while q:
        c = q.popleft()
        for d, (di, dj) in DIRS.items():
            if known[c[0]][c[1]] & d:
                continue
            n = (c[0] + di, c[1] + dj)
            if 0 <= n[0] < size and 0 <= n[1] < size and n not in dist:
                dist[n] = dist[c] + 1
                q.append(n)
    ops = 0
    cur = robot.cell_from_odom()
    while cur != target and cur in dist and ops < max_ops:
        ops += 1
        curd = dist[cur]
        for d, (di, dj) in DIRS.items():
            if known[cur[0]][cur[1]] & d:
                continue
            n = (cur[0] + di, cur[1] + dj)
            if n in dist and dist[n] == curd - 1:
                rel = rel_between(robot.heading_from_yaw(), d)
                robot.move_step(rel, robot.heading_from_yaw())
                break
        cur = robot.cell_from_odom()
    print(f"[回起点] 到达 {cur}，用了 {ops} 步")


def ascii_of(walls, size=16):
    from maze_model import N, E, S, W
    rows = []
    for j in reversed(range(size)):
        top = "".join("+" + ("---" if walls[i][j] & N else "   ") for i in range(size))
        mid = "".join(("|" if walls[i][j] & W else " ") + "   " for i in range(size))
        rows += [top + "+", mid + ("|" if walls[size - 1][j] & E else " ")]
    rows.append("".join("+" + ("---" if walls[i][0] & S else "   ") for i in range(size)) + "+")
    return "\n".join(rows)


def _run():
    try:
        return main()
    except Exception as exc:
        print("\n[异常]", exc)
        sys.stdout.flush()
        os._exit(1)


if __name__ == "__main__":
    _run()
