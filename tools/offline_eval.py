"""离线评测：在 .TQD / .world 迷宫上跑 Bulebule 风格基线，输出关键指标。

用法:
    python3 tools/offline_eval.py                 # maze1..3
    python3 tools/offline_eval.py --noise 0.02    # 加 2% 传感器误判
    python3 tools/offline_eval.py --verbose
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from maze_model import N, E, S, W, DIRS, from_tqd, from_world
from robot import SimRobot
from baseline_floodfill import FloodFillSearch

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
GOALS = [(7, 7), (7, 8), (8, 7), (8, 8)]

# 计分/物理参数（来自 config.ini 与竞赛规则）
BLOCK = 0.178          # 每格行程 m
V_X = 0.10             # 直行速度 m/s
W_Z = 0.20             # 转向角速度 rad/s
T_90 = (3.1415926 / 2) / W_Z
T_180 = 3.1415926 / W_Z


def est_seconds(stats):
    """按竞赛默认速度折算运动耗时（不含感知/计算开销）。"""
    return stats.moves * BLOCK / V_X + stats.turn90 * T_90 + stats.turn180 * T_180


def run_one(maze, name, noise=0.0, seed=0, verbose=False):
    truth = maze
    robot = SimRobot(truth, noise=noise, seed=seed)
    robot.set_pose((0, 0), N)
    algo = FloodFillSearch(robot, goals=GOALS, verbose=verbose)
    stats = algo.explore()

    run_path = algo.build_run_path()
    optimal = truth.bfs_path_len((0, 0), GOALS)

    # 冲刺运动统计（等价于执行 F/L/R 序列）
    CW = [N, E, S, W]
    heading = algo.heading0
    moves = turns90 = turns180 = 0
    for (ni, nj, d) in run_path:
        delta = (CW.index(d) - CW.index(heading)) % 4
        if delta == 0:
            moves += 1
        elif delta == 2:
            turns180 += 1
            moves += 1
        else:
            turns90 += 1
            moves += 1
        heading = d

    exp_t = est_seconds(stats)
    run_t = moves * BLOCK / V_X + turns90 * T_90 + turns180 * T_180
    # 竞赛排障时间 = 迷宫时间/30 + 运行时间 - 2s(无碰触奖励)
    score = exp_t / 30.0 + run_t - 2.0

    # 只统计"已访问过的格子"里墙壁记忆错误（未访问格没有信息，不算错）
    know_err = sum(1 for (i, j) in algo.visited
                   if algo.known[i][j] != truth.walls[i][j])

    return dict(name=name, optimal=optimal, run_len=len(run_path),
                explore_moves=stats.moves, explore_turn90=stats.turn90,
                explore_turn180=stats.turn180, sense=stats.sense,
                reflood=stats.reflood, visited=len(algo.visited),
                exp_sec=exp_t, run_moves=moves, run_turn90=turns90,
                run_turn180=turns180, run_sec=run_t, score=score,
                wall_err=know_err)


def fmt(r):
    return (f"{r['name']:8s} | 最短 {r['optimal']:2d} | 冲刺 {r['run_len']:2d}"
            f" | 探索 {r['explore_moves']:4d}格/{r['explore_turn90']:3d}转90°/{r['explore_turn180']:2d}转180°"
            f" | 感知 {r['sense']:3d} | 访问 {r['visited']:3d}/256"
            f" | 探索 {r['exp_sec']:7.1f}s | 运行 {r['run_sec']:6.1f}s"
            f" | 排障 {r['score']:7.1f}s | 已访问区墙错误 {r['wall_err']:2d}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--noise", type=float, default=0.0)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--verbose", action="store_true")
    ap.add_argument("--source", choices=["tqd", "world"], default="world")
    ap.add_argument("mazes", nargs="*", default=["1", "2", "3"])
    args = ap.parse_args()

    print("== Bulebule 风格洪水填充基线 · 离线评测 ==")
    print(f"   迷宫来源={args.source}  传感器误判率={args.noise}")
    print("-" * 150)
    results = []
    for n in args.mazes:
        if args.source == "world":
            maze = from_world(os.path.join(ROOT, "world", f"maze{n}.world"))
        else:
            maze = from_tqd(os.path.join(ROOT, "TQD", f"maze{n}.TQD"))
        r = run_one(maze, f"maze{n}", args.noise, args.seed, args.verbose)
        results.append(r)
        print(fmt(r))
    print("-" * 150)
    if results:
        avg_exp = sum(r["exp_sec"] for r in results) / len(results)
        avg_run = sum(r["run_sec"] for r in results) / len(results)
        print(f"平均: 探索 {avg_exp:.1f}s  运行 {avg_run:.1f}s")


if __name__ == "__main__":
    main()
