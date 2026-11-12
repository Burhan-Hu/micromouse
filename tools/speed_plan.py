"""速度规划：把 F/L/R 走位串编译成带限速的多航点轨迹。

为什么需要
    逐格冲刺时每到一个格心就要停车再起步，即使直道也一样，
    实际平均速度远低于 vmax。真实比赛的做法是：
        * 直道全速；
        * 过弯前提前减速（弯道限速）；
        * 转弯过程中不停车，只降速。

模型（与实测标定一致）
    * 加速/制动加速度上限  a = 0.8 m/s^2（轮子加速度受限，实测等效）
    * 直道最大速度        vmax
    * 90° 过弯限速        v90   —— 转弯瞬间车速不超过它
    （原地转向时真正的问题是"边转边走会撞墙"，所以我们对每段转弯
      在弯前把速度压到 v90，弯后重新加速。）

产物
    segments = [(length_m, v_entry, v_exit, turn_after_deg), ...]
    由 RosDriver.run_segments() 执行：每段用"速度梯形"跑完，
    转弯处用点到点转向并保持轻微前进来缩短时间。
"""

import math

CELL = 0.18


def plan(path_dirs, start_pos=(0, 0), start_heading=None,
         vmax=1.0, v90=0.35, accel=0.8):
    """把方向序列编译为 [(格数, 方向, 该段允许最高速), ...]。

    限速规则：每段的可达最高速 = min(vmax, sqrt(v90^2 + 2*a*L))
    即"从弯道限速出发，用 a 加速完这段能到多快"。
    段间转弯由执行器负责（见 RosDriver.run_segments）。
    """
    runs = []
    for d in path_dirs:
        if runs and runs[-1][1] == d:
            runs[-1][0] += 1
        else:
            runs.append([1, d])

    out = []
    for n, d in runs:
        length = n * CELL
        reach = math.sqrt(v90 * v90 + 2.0 * accel * length)
        out.append((n, d, min(vmax, reach)))
    return out


def estimate_time(segments, vmax, accel=0.8):
    """按梯形速度曲线估算总用时（用于离线对比）。"""
    t = 0.0
    for n, d, v in segments:
        L = n * CELL
        v = min(v, vmax)
        # 从 0 加速到 v 再减速到 0 的距离
        d_acc = v * v / (2 * accel)
        if 2 * d_acc >= L:
            t += 2 * math.sqrt(L / accel)
        else:
            t += (L - 2 * d_acc) / v + 2 * (v / accel)
    return t
