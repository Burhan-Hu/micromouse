"""定位漂移鲁棒性评测：物理位置精确，但里程计估计会漂。

用来回答"如果轮子打滑/编码器漏计一格，算法还能不能跑对"。
三套算法用同一套漂移模型、同一随机种子，横向对比。

用法:
    python3 tools/robust_eval.py                 # maze1，漂移率 0/1%/3%
    python3 tools/robust_eval.py --mazes 1 2 3
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from maze_model import N, E, S, W, DIRS, from_world
from robot import PoseNoiseRobot
from baseline_floodfill import FloodFillSearch
from baseline_mms import MMSSearch, turn_left, turn_right
from fused_search import FusedSearch

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
GOALS = [(7, 7), (7, 8), (8, 7), (8, 8)]


def check(truth, path_dirs, start=(0, 0), heading0=N):
    """沿真实迷宫检查方向序列是否合法且到达目标。"""
    pos, heading = start, heading0
    for d in path_dirs:
        if truth.walls[pos[0]][pos[1]] & d:
            return False, pos
        di, dj = DIRS[d]
        pos = (pos[0] + di, pos[1] + dj)
    return pos in set(GOALS), pos


def build(algo_name, robot):
    if algo_name == "bulebule":
        a = FloodFillSearch(robot, goals=GOALS)
    elif algo_name == "mms":
        a = MMSSearch(robot, goals=GOALS)
    elif algo_name == "fused+reloc":
        a = FusedSearch(robot, goals=GOALS, relocalize=True)
    else:
        a = FusedSearch(robot, goals=GOALS)
    # 用"里程计估计"覆盖算法自身的位置/朝向更新
    # （fused+reloc 例外：让它自己的重定位结果生效，否则等于关掉了重定位）
    if algo_name != "fused+reloc":
        def apply_est(rel, _a=a, _r=robot):
            _a.pos = _r.est
            _a.heading = _r.est_heading
        a._apply_step = apply_est
    return a


def run(algo_name, truth, slip, seed=0, sensor_noise=0.0):
    robot = PoseNoiseRobot(truth, cell_slip=slip, noise=sensor_noise, seed=seed)
    robot.set_pose((0, 0), N)
    a = build(algo_name, robot)
    try:
        if algo_name.startswith("fused"):
            dirs = [d for (_, _, d) in a.solve()]
        elif algo_name == "bulebule":
            a.explore()
            dirs = [d for (_, _, d) in a.build_run_path()]
        else:
            a.explore()
            seq = a.run_sequence()
            hd, dirs = N, []
            for ch in seq:
                if ch == "L":
                    hd = turn_left(hd)
                elif ch == "R":
                    hd = turn_right(hd)
                elif ch == "F":
                    dirs.append(hd)
    except Exception as exc:
        return dict(ok=False, err=str(exc), cells=0, moves=0)
    ok, end = check(truth, dirs)
    real_moves = sum(1 for d in dirs)
    return dict(ok=ok, cells=real_moves, err="", end=end)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mazes", nargs="*", default=["1"])
    ap.add_argument("--slips", nargs="*", type=float, default=[0.0, 0.01, 0.03])
    ap.add_argument("--seeds", nargs="*", type=int, default=[0, 1, 2])
    ap.add_argument("--noise", type=float, default=0.0)
    args = ap.parse_args()

    print("== 定位漂移鲁棒性（物理精确 / 里程计漂移）==")
    print(f"   传感器误判率={args.noise}")
    for m in args.mazes:
        truth = from_world(os.path.join(ROOT, "world", f"maze{m}.world"))
        opt = truth.bfs_path_len((0, 0), GOALS)
        print("-" * 96)
        print(f"maze{m}  理论最短 {opt} 格")
        for slip in args.slips:
            row = []
            for algo in ("bulebule", "mms", "fused", "fused+reloc"):
                oks, lens = 0, []
                for seed in args.seeds:
                    r = run(algo, truth, slip, seed, args.noise)
                    oks += 1 if r["ok"] else 0
                    if r["ok"]:
                        lens.append(r["cells"])
                avg = (sum(lens) / len(lens)) if lens else 0
                row.append(f"{algo}={oks}/{len(args.seeds)}次合法(均{avg:.0f}格)")
            print(f"  漂移率 {slip:>4.1%}: " + "  |  ".join(row))
    print("-" * 96)


if __name__ == "__main__":
    main()
