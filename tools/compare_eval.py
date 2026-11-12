"""两套基线对比：Bulebule 风格洪水 vs mms 风格洪水（含探索偏置与路径压缩）。

用法:
    python3 tools/compare_eval.py                # maze1..3
    python3 tools/compare_eval.py --noise 0.02
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from maze_model import N, E, S, W, DIRS, from_world, from_tqd
from robot import SimRobot
from baseline_floodfill import FloodFillSearch
from baseline_mms import MMSSearch, compress_path, turn_left, turn_right
from fused_search import FusedSearch

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
GOALS = [(7, 7), (7, 8), (8, 7), (8, 8)]

BLOCK, V_X, W_Z = 0.178, 0.10, 0.20
T90 = (3.1415926 / 2) / W_Z
T180 = 3.1415926 / W_Z


def secs(moves, t90, t180):
    return moves * BLOCK / V_X + t90 * T90 + t180 * T180


def simulate_sequence(truth, seq, start=(0, 0), heading0=N, goals=None):
    """在真实迷宫上执行 F/L/R 序列，返回 (合法?, 终点, 格数, 90°数)。"""
    goals = set(goals or GOALS)
    pos, heading = start, heading0
    moves = t90 = 0
    for ch in seq:
        if ch == "L":
            heading = turn_left(heading)
            t90 += 1
        elif ch == "R":
            heading = turn_right(heading)
            t90 += 1
        elif ch == "F":
            if truth.walls[pos[0]][pos[1]] & heading:
                return False, pos, moves, t90
            di, dj = DIRS[heading]
            pos = (pos[0] + di, pos[1] + dj)
            moves += 1
    return (pos in goals), pos, moves, t90


def run_bulebule(truth, noise, seed, verbose):
    robot = SimRobot(truth, noise=noise, seed=seed)
    robot.set_pose((0, 0), N)
    algo = FloodFillSearch(robot, goals=GOALS, verbose=verbose)
    st = algo.explore()
    run_path = algo.build_run_path()
    heading = algo.heading0
    moves = t90 = t180 = 0
    for (ni, nj, d) in run_path:
        delta = ([N, E, S, W].index(d) - [N, E, S, W].index(heading)) % 4
        if delta == 0:
            moves += 1
        elif delta == 2:
            t180 += 1
            moves += 1
        else:
            t90 += 1
            moves += 1
        heading = d
    err = sum(1 for (i, j) in algo.visited if algo.known[i][j] != truth.walls[i][j])
    return dict(exp_moves=st.moves, exp_t90=st.turn90, exp_t180=st.turn180,
                sense=st.sense, visited=len(algo.visited), wall_err=err,
                run_moves=moves, run_t90=t90, run_t180=t180,
                run_ok=True, seq=run_path and len(run_path))


def run_mms(truth, noise, seed, verbose, compress=True):
    robot = SimRobot(truth, noise=noise, seed=seed)
    robot.set_pose((0, 0), N)
    algo = MMSSearch(robot, goals=GOALS, verbose=verbose, compress=compress)
    st = algo.explore()
    seq = algo.run_sequence()
    ok, end, moves, t90 = simulate_sequence(truth, seq)
    raw_moves = seq.count("F")
    err = sum(1 for (i, j) in algo.explored if algo.walls[i][j] != truth.walls[i][j])
    return dict(exp_moves=st.moves, exp_t90=st.turn90, exp_t180=st.turn180,
                sense=st.sense, visited=len(algo.explored), wall_err=err,
                run_moves=raw_moves, run_t90=t90, run_t180=0,
                run_ok=ok, end=end, seq_len=len(seq))


def run_mms_floodrun(truth, noise, seed, verbose):
    """mms 探索 + 用已知地图重新洪水求最短路冲刺（用来区分"探索策略"与"路线选择"）。"""
    robot = SimRobot(truth, noise=noise, seed=seed)
    robot.set_pose((0, 0), N)
    algo = MMSSearch(robot, goals=GOALS, verbose=verbose, compress=False)
    st = algo.explore()
    dist = algo.flood()
    # 从起点沿距离场下降到目标，编译成 F/L/R 序列
    pos, heading = (0, 0), N
    seq = []
    guard = 0
    moves = t90 = 0
    while dist[pos[0]][pos[1]] != 0:
        guard += 1
        if guard > 1000:
            break
        cur = dist[pos[0]][pos[1]]
        step_dir = None
        for d in (N, E, S, W):
            if algo.walls[pos[0]][pos[1]] & d:
                continue
            di, dj = DIRS[d]
            nxt = (pos[0] + di, pos[1] + dj)
            if algo.inside(*nxt) and dist[nxt[0]][nxt[1]] == cur - 1:
                step_dir = d
                break
        if step_dir is None:
            break
        delta = ([N, E, S, W].index(step_dir) - [N, E, S, W].index(heading)) % 4
        if delta == 1:
            seq.append("R")
            t90 += 1
        elif delta == 3:
            seq.append("L")
            t90 += 1
        elif delta == 2:
            seq += ["R", "R"]
            t90 += 2
        seq.append("F")
        moves += 1
        di, dj = DIRS[step_dir]
        pos = (pos[0] + di, pos[1] + dj)
        heading = step_dir
    ok, end, m2, t2 = simulate_sequence(truth, "".join(seq))
    err = sum(1 for (i, j) in algo.explored if algo.walls[i][j] != truth.walls[i][j])
    return dict(exp_moves=st.moves, exp_t90=st.turn90, exp_t180=st.turn180,
                sense=st.sense, visited=len(algo.explored), wall_err=err,
                run_moves=m2, run_t90=t2, run_t180=0, run_ok=ok, end=end)


def run_fused(truth, noise, seed, verbose, verify=True, label=""):
    robot = SimRobot(truth, noise=noise, seed=seed)
    robot.set_pose((0, 0), N)
    algo = FusedSearch(robot, goals=GOALS, verbose=verbose, verify=verify)
    exp = algo.explore()
    ver = PhaseStatsCopy(algo)
    algo.verify()
    run_path = algo.build_run_path()
    heading = N
    moves = t90 = t180 = 0
    seq = []
    for (ni, nj, d) in run_path:
        delta = ([N, E, S, W].index(d) - [N, E, S, W].index(heading)) % 4
        if delta == 1:
            seq.append("R")
        elif delta == 3:
            seq.append("L")
        elif delta == 2:
            seq += ["R", "R"]
        seq.append("F")
        if delta == 0:
            moves += 1
        elif delta == 2:
            t180 += 1
            moves += 1
        else:
            t90 += 1
            moves += 1
        heading = d
    ok, end, m2, t2 = simulate_sequence(truth, "".join(seq))
    err = sum(1 for (i, j) in algo.explored if algo.walls[i][j] != truth.walls[i][j])
    return dict(exp_moves=exp.moves + ver["moves"], exp_t90=exp.turn90 + ver["t90"],
                exp_t180=exp.turn180 + ver["t180"], sense=exp.sense + ver["sense"],
                visited=len(algo.explored), wall_err=err,
                run_moves=moves, run_t90=t90, run_t180=t180, run_ok=ok,
                extra=ver)


def PhaseStatsCopy(algo):
    v = algo.stats_verify
    return dict(moves=v.moves, t90=v.turn90, t180=v.turn180, sense=v.sense,
                corrections=v.corrections)


def fmt_row(tag, r, optimal):
    exp_t = secs(r["exp_moves"], r["exp_t90"], r["exp_t180"])
    run_t = secs(r["run_moves"], r["run_t90"], r["run_t180"])
    score = exp_t / 30.0 + run_t - 2.0
    eff = optimal / r["run_moves"] if r["run_moves"] else 0
    return (f"  {tag:9s}| 探索 {r['exp_moves']:4d}格/{r['exp_t90']:3d}×90°/{r['exp_t180']:2d}×180°"
            f" | 感知 {r['sense']:3d} | 访问 {r['visited']:3d}"
            f" | 冲刺 {r['run_moves']:3d}格/{r['run_t90']:3d}转"
            f" | 效率 {eff:6.1%} | 排障 {score:7.1f}s"
            f" | 墙误 {r['wall_err']:2d} {'OK' if r['run_ok'] else '路径非法!'}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--noise", type=float, default=0.0)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--verbose", action="store_true")
    ap.add_argument("--source", choices=["world", "tqd"], default="world")
    ap.add_argument("mazes", nargs="*", default=["1", "2", "3"])
    args = ap.parse_args()

    print("== 基线对比：Bulebule 风格  vs  mms 风格(探索偏置+路径压缩) ==")
    print(f"   传感器误判率={args.noise}   时间按 直行0.10m/s、转向0.20rad/s 折算")
    for n in args.mazes:
        if args.source == "world":
            truth = from_world(os.path.join(ROOT, "world", f"maze{n}.world"))
        else:
            truth = from_tqd(os.path.join(ROOT, "TQD", f"maze{n}.TQD"))
        optimal = truth.bfs_path_len((0, 0), GOALS)
        print("-" * 150)
        print(f"maze{n}  理论最短 {optimal} 格")
        print(fmt_row("Bulebule", run_bulebule(truth, args.noise, args.seed, args.verbose), optimal))
        print(fmt_row("mms", run_mms(truth, args.noise, args.seed, args.verbose), optimal))
        print(fmt_row("mms+洪水", run_mms_floodrun(truth, args.noise, args.seed, args.verbose), optimal))
        print(fmt_row("融合", run_fused(truth, args.noise, args.seed, args.verbose), optimal))
        print(fmt_row("融合-校验", run_fused(truth, args.noise, args.seed, args.verbose,
                                          verify=False), optimal))
    print("-" * 150)


if __name__ == "__main__":
    main()
