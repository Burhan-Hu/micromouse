"""TQD-OC V2.0 迷宫机器人竞速赛道参赛程序

算法：融合洪水填充
    阶段 A  探索：mms 式"探索偏置"（优先未访问且距离不增的格子）走向中心，
                 到达目标后不立即收工，而是检查"起点→目标"已知最短路是否全程
                 已访问；未满足就把路径上第一个未访问格作为临时目标补探。
    阶段 B  校验：回起点，沿最短路逐格重扫（3 次采样取多数），发现新墙即重算；
                 目标不可达时逐面"假设该墙不存在"并实地验证，假墙会被重新观测
                 到从而自动恢复。因此冲刺路径不会穿墙。
    阶段 C  冲刺：在最终地图上求最短路，以更高速度执行；减速曲线由
                 goto_point 的 v = min(vmax, k*dist) 自动给出。

接口（与评测系统一致）：
    /scan    LaserScan  360 线，角度 -90°..+90°
    /odom    Odometry   位置(m) + 四元数
    /cmd_vel Twist      linear.x 前进, angular.z 转向(正值左转)

坐标约定：单元格边长 0.18 m；起点格中心自动标定为原点；
          朝向 yaw = euler_z + pi ∈ [0, 2pi)，北 = 4.71 / 东 = 3.14 / 南 = 1.57 / 西 = 0。
"""

import math
import os
import sys
import threading
import time
from collections import deque
from configparser import ConfigParser

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

# ---------------------------------------------------------------- 常量
N, E, S, W = 1, 2, 4, 8
DIRS = {N: (0, 1), E: (1, 0), S: (0, -1), W: (-1, 0)}
OPP = {N: S, E: W, S: N, W: E}
CW = [N, E, S, W]
FRONT, LEFT, RIGHT, BACK = 0, 1, 2, 3
INF = 10 ** 6
SIZE = 16
GOALS = [(7, 7), (7, 8), (8, 7), (8, 8)]

CELL = 0.18                 # 单元格边长 m
DETECT = 0.15               # "有墙"判定距离（侧墙实测≈0.084，无墙≥0.17）

EXPLORE_SPEED = 0.22        # 探索/校验直行速度
# 冲刺速度：本仿真模型的轮子会对指令速度产生"空转"，实际车体速度远低于
# cmd_vel（实测 0.5 m/s 指令对应约 1.0 m/s 车体速度），且 URDF 的
# wheel_diameter=0.02 m / max_wheel_acceleration=1.0 使逐格控制在高速下抖动。
# 为保证可复现性，这里保守取 0.20（等于我们做过完整实测的速度）。
RUN_SPEED = 0.20
TURN_SPEED = 1.2            # 转向角速度 rad/s


def turn_right(d):
    return CW[(CW.index(d) + 1) % 4]


def turn_left(d):
    return CW[(CW.index(d) + 3) % 4]


def rel_to_abs(heading, rel):
    if rel == FRONT:
        return heading
    if rel == RIGHT:
        return turn_right(heading)
    if rel == BACK:
        return OPP[heading]
    return turn_left(heading)


def abs_to_world(d):
    return {N: math.pi / 2, E: 0.0, S: -math.pi / 2, W: math.pi}[d]


# ---------------------------------------------------------------- 底层驱动
class Drive:
    """ROS 接口 + 点到点闭环运动控制。"""

    BANDS = {'l': (330, 359), 'fl': (250, 290), 'f': (165, 195),
             'fr': (70, 110), 'r': (0, 30)}

    def __init__(self, config_path=None):
        self.kp = self._load_kp(config_path)
        self.l_dis = self.fl_dis = self.f_dis = self.fr_dis = self.r_dis = 0.0
        self.x = self.y = 0.0
        self.yaw = 0.0
        self.odom_ready = False
        self.x0 = self.y0 = 0.0             # 起点格中心（自动标定）
        self.speed_x = EXPLORE_SPEED
        self.speed_z = TURN_SPEED

        self.node = Node('micromouse_contest')
        self.node.create_subscription(LaserScan, '/scan', self._laser, 10)
        self.node.create_subscription(Odometry, '/odom', self._odom, 10)
        self.pub = self.node.create_publisher(Twist, '/cmd_vel', 10)
        self.rate = self.node.create_rate(100.0)

    @staticmethod
    def _load_kp(path):
        if path and os.path.exists(path):
            c = ConfigParser()
            with open(path) as f:
                c.read_file(f)
            if c.has_option('threshold', 'kp'):
                return float(c.get('threshold', 'kp'))
        return 10.0

    # ---- ROS 回调 ----
    def _laser(self, msg):
        r = msg.ranges
        lo = msg.range_min if msg.range_min > 0 else 0.0

        def band(key):
            a, b = self.BANDS[key]
            vals = sorted(v for v in r[a:b + 1] if v == v and v > lo)
            if not vals:
                return float('inf')
            # 取第 3 小值，规避仿真中偶发的单条射线野值
            return vals[min(2, len(vals) - 1)]

        self.l_dis = band('l')
        self.fl_dis = band('fl')
        self.f_dis = band('f')
        self.fr_dis = band('fr')
        self.r_dis = band('r')

    def _odom(self, msg):
        p = msg.pose.pose.position
        self.x, self.y = p.x, p.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = (math.atan2(siny, cosy) + math.pi) % (2 * math.pi)
        if not self.odom_ready and (self.x or self.y):
            self.odom_ready = True

    # ---- 生命周期 ----
    def spin_in_thread(self):
        threading.Thread(target=rclpy.spin, args=(self.node,), daemon=True).start()
        t0 = time.time()
        while time.time() - t0 < 20:
            if self.odom_ready and self.l_dis > 0:
                return True
            time.sleep(0.05)
        return False

    def calibrate_origin(self):
        self.x0, self.y0 = self.x, self.y

    def publish(self, vx, wz):
        m = Twist()
        m.linear.x = float(vx)
        m.angular.z = float(wz)
        self.pub.publish(m)

    def stop(self):
        for _ in range(6):
            self.publish(0.0, 0.0)
            self.rate.sleep()

    # ---- 位姿 ----
    @staticmethod
    def wrap(a):
        while a <= -math.pi:
            a += 2 * math.pi
        while a > math.pi:
            a -= 2 * math.pi
        return a

    def world_yaw(self):
        return self.wrap(self.yaw - math.pi)

    def cell_from_odom(self):
        return (int(round((self.x - self.x0) / CELL)),
                int(round((self.y - self.y0) / CELL)))

    def heading_from_yaw(self):
        y = self.yaw
        if abs(y - 4.7124) < 0.8:
            return N
        if abs(y - 3.1416) < 0.8:
            return E
        if abs(y - 1.5708) < 0.8:
            return S
        return W

    def read_walls(self):
        return (self.l_dis < DETECT, self.f_dis < DETECT, self.r_dis < DETECT)

    # ---- 运动 ----
    def turn_to_dir(self, direction, timeout=4.0):
        target = abs_to_world(direction)
        t0 = time.time()
        while rclpy.ok():
            err = self.wrap(target - self.world_yaw())
            if abs(err) < 0.04:
                break
            if time.time() - t0 > timeout:
                break
            self.publish(0.0, max(-self.speed_z, min(self.speed_z, 2.5 * err)))
            self.rate.sleep()
        self.stop()

    def goto_point(self, tx, ty, timeout=8.0, tol=0.006):
        t0 = time.time()
        while rclpy.ok():
            ex, ey = tx - self.x, ty - self.y
            dist = math.hypot(ex, ey)
            if dist < tol:
                break
            if time.time() - t0 > timeout:
                break
            err = self.wrap(math.atan2(ey, ex) - self.world_yaw())
            wz = max(-1.0, min(1.0, 3.0 * err)) * min(1.0, dist / 0.06)
            if abs(err) > 0.9:
                self.publish(0.0, wz)                      # 方向差太多先原地转
            else:
                v = max(0.03, min(self.speed_x, 1.6 * dist))  # 近目标自动减速
                self.publish(v, wz)
            self.rate.sleep()
        self.stop()

    def goto_cell(self, i, j, **kw):
        self.goto_point(self.x0 + CELL * i, self.y0 + CELL * j, **kw)

    def move_step(self, rel, heading):
        d = rel_to_abs(heading, rel)
        if rel != FRONT:
            self.turn_to_dir(d)
        i, j = self.cell_from_odom()
        di, dj = DIRS[d]
        self.goto_cell(i + di, j + dj)
        if abs(self.wrap(abs_to_world(d) - self.world_yaw())) > 0.07:
            self.turn_to_dir(d)

    def turn_only(self, direction):
        self.turn_to_dir(direction)


# ---------------------------------------------------------------- 算法
class Solver:
    def __init__(self, drive, goals=None):
        self.drive = drive
        self.goals = list(goals or GOALS)
        self.goal_set = set(self.goals)
        self.start = (0, 0)

        self.walls = [[0] * SIZE for _ in range(SIZE)]
        for i in range(SIZE):
            self.walls[i][0] |= S
            self.walls[i][SIZE - 1] |= N
            self.walls[0][i] |= W
            self.walls[SIZE - 1][i] |= E
        self.explored = set()
        self.pos = (0, 0)
        self.heading = N
        self.dist = None
        self.trace = []
        self.repeat = 1
        self.n_moves = 0

    # ---- 地图 ----
    def inside(self, i, j):
        return 0 <= i < SIZE and 0 <= j < SIZE

    def _place_wall(self, i, j, d):
        self.walls[i][j] |= d
        ni, nj = i + DIRS[d][0], j + DIRS[d][1]
        if self.inside(ni, nj):
            self.walls[ni][nj] |= OPP[d]

    def _clear_wall(self, i, j, d):
        ni, nj = i + DIRS[d][0], j + DIRS[d][1]
        if not self.inside(ni, nj) or not (self.walls[i][j] & d):
            return False
        self.walls[i][j] &= ~d
        self.walls[ni][nj] &= ~OPP[d]
        return True

    def flood(self, targets=None):
        targets = targets or self.goals
        dist = [[INF] * SIZE for _ in range(SIZE)]
        q = deque()
        for (i, j) in targets:
            if self.inside(i, j):
                dist[i][j] = 0
                q.append((i, j))
        while q:
            i, j = q.popleft()
            nd = dist[i][j] + 1
            for d, (di, dj) in DIRS.items():
                if self.walls[i][j] & d:
                    continue
                ni, nj = i + di, j + dj
                if self.inside(ni, nj) and dist[ni][nj] == INF:
                    dist[ni][nj] = nd
                    q.append((ni, nj))
        self.dist = dist
        return dist

    # ---- 感知 ----
    def sense(self, correct=False, repeat=1):
        if repeat > 1:
            votes = [self.drive.read_walls() for _ in range(repeat)]
            wl = sum(v[0] for v in votes) * 2 > repeat
            wf = sum(v[1] for v in votes) * 2 > repeat
            wr = sum(v[2] for v in votes) * 2 > repeat
        else:
            wl, wf, wr = self.drive.read_walls()
        changed = False
        i, j = self.pos
        for has, rel in ((wf, FRONT), (wl, LEFT), (wr, RIGHT)):
            d = rel_to_abs(self.heading, rel)
            if has:
                if not (self.walls[i][j] & d):
                    self._place_wall(i, j, d)
                    changed = True
            elif correct and self._clear_wall(i, j, d):
                changed = True
        self.explored.add(self.pos)
        return changed

    # ---- 决策 ----
    def _can_move(self, rel):
        d = rel_to_abs(self.heading, rel)
        if self.walls[self.pos[0]][self.pos[1]] & d:
            return False
        ni, nj = self.pos[0] + DIRS[d][0], self.pos[1] + DIRS[d][1]
        return self.inside(ni, nj)

    def _next_pos(self, rel):
        d = rel_to_abs(self.heading, rel)
        return (self.pos[0] + DIRS[d][0], self.pos[1] + DIRS[d][1])

    def _dist_of(self, rel):
        if not self._can_move(rel):
            return INF
        i, j = self._next_pos(rel)
        return self.dist[i][j]

    def biased_step(self):
        """mms 探索偏置：优先未访问且距离不增的相邻格。"""
        cur = self.dist[self.pos[0]][self.pos[1]]
        best, best_d = None, INF
        found = False
        for rel in (FRONT, LEFT, RIGHT, BACK):
            if not self._can_move(rel):
                continue
            d = self._dist_of(rel)
            unexplored = self._next_pos(rel) not in self.explored
            if unexplored and d <= cur and (not found or d < best_d):
                best_d, best, found = d, rel, True
            elif not found and d < best_d:
                best_d, best = d, rel
        return best

    def descent_step(self):
        cur = self.dist[self.pos[0]][self.pos[1]]
        for rel in (FRONT, LEFT, RIGHT, BACK):
            if self._can_move(rel) and self._dist_of(rel) == cur - 1:
                return rel
        best, best_d = None, INF
        for rel in (FRONT, LEFT, RIGHT, BACK):
            if self._can_move(rel):
                d = self._dist_of(rel)
                if d < best_d:
                    best_d, best = d, rel
        return best

    def _apply(self, rel):
        d = rel_to_abs(self.heading, rel)
        self.pos = (self.pos[0] + DIRS[d][0], self.pos[1] + DIRS[d][1])
        self.heading = d
        self.n_moves += 1

    def _step_once(self, rel):
        self.drive.move_step(rel, self.heading)
        self._apply(rel)
        self.trace.append(self.pos)

    # ---- 行走 ----
    def walk(self, targets, bias=False, force_sense=False, correct=False,
             repeat=1, max_steps=4000):
        tset = set(targets)
        self.flood(targets)
        steps = 0
        while rclpy.ok() and steps < max_steps:
            if (force_sense or correct or self.pos not in self.explored) \
                    and self.sense(correct=correct, repeat=repeat):
                self.flood(targets)
            if self.pos in tset:
                return True
            rel = self.biased_step() if bias else self.descent_step()
            if rel is None:
                return False
            self._step_once(rel)
            steps += 1
        return False

    def known_path(self):
        dist = self.flood(self.goals)
        i, j = self.start
        if dist[i][j] >= INF:
            return None
        path = []
        guard = 0
        while dist[i][j] != 0:
            guard += 1
            if guard > 4000:
                return None
            cur = dist[i][j]
            for d in CW:
                if self.walls[i][j] & d:
                    continue
                ni, nj = i + DIRS[d][0], j + DIRS[d][1]
                if self.inside(ni, nj) and dist[ni][nj] == cur - 1:
                    path.append((ni, nj))
                    i, j = ni, nj
                    break
            else:
                return None
        return path

    def path_fully_known(self):
        p = self.known_path()
        return p is not None and all(c in self.explored for c in p)

    # ---- 阶段 A：探索 ----
    def explore(self, max_rounds=80):
        self.flood(self.goals)
        for _ in range(max_rounds):
            self.walk(self.goals, bias=True, repeat=self.repeat)
            if self.pos in self.goal_set and self.path_fully_known():
                return True
            p = self.known_path()
            if p is None:
                return False
            target = next((c for c in p if c not in self.explored), None)
            if target is None:
                return False
            self.walk([target], repeat=self.repeat)
        return False

    # ---- 阶段 B：校验 ----
    def _wall_count(self):
        return sum(v.bit_count() for row in self.walls for v in row)

    def repair_candidates(self, limit=8):
        self.flood(self.goals)
        d_goal = self.dist
        d_home = [[INF] * SIZE for _ in range(SIZE)]
        q = deque([self.start])
        d_home[0][0] = 0
        while q:
            i, j = q.popleft()
            for d, (di, dj) in DIRS.items():
                if self.walls[i][j] & d:
                    continue
                ni, nj = i + di, j + dj
                if self.inside(ni, nj) and d_home[ni][nj] == INF:
                    d_home[ni][nj] = d_home[i][j] + 1
                    q.append((ni, nj))
        cands = []
        for i in range(SIZE):
            for j in range(SIZE):
                if d_home[i][j] >= INF:
                    continue
                for d, (di, dj) in DIRS.items():
                    if not (self.walls[i][j] & d):
                        continue
                    ni, nj = i + di, j + dj
                    if not self.inside(ni, nj) or d_goal[ni][nj] >= INF:
                        continue
                    cands.append((d_home[i][j] + 1 + d_goal[ni][nj], i, j, d))
        cands.sort()
        return [(i, j, d) for _, i, j, d in cands[:limit]]

    def _walk_path_sensing(self):
        before = self._wall_count()
        self.walk(self.goals, force_sense=True, repeat=3)
        return self._wall_count() - before

    def verify(self, max_rounds=8):
        cleared = set()
        for _ in range(max_rounds):
            if self.pos != self.start:
                self.walk([self.start])
            if self.known_path() is not None:
                if self._walk_path_sensing() == 0 and self.path_fully_known():
                    return True
                continue
            progressed = False
            for (i, j, d) in self.repair_candidates():
                if (i, j, d) in cleared:
                    continue
                cleared.add((i, j, d))
                self._clear_wall(i, j, d)
                if self.known_path() is None:
                    self._place_wall(i, j, d)      # 清了也没用，立刻还原
                    continue
                self._walk_path_sensing()          # 实地验证：假墙会被重新观测到
                progressed = True
                if self.pos in self.goal_set and self.path_fully_known():
                    return True
            if not progressed:
                break
        return self.path_fully_known()

    def remap(self):
        for i in range(SIZE):
            for j in range(SIZE):
                self.walls[i][j] = 0
        for i in range(SIZE):
            self.walls[i][0] |= S
            self.walls[i][SIZE - 1] |= N
            self.walls[0][i] |= W
            self.walls[SIZE - 1][i] |= E
        self.explored.clear()
        self.trace.clear()
        self.repeat = 3
        self.walk([self.start], repeat=3)
        self.explore()
        self.repeat = 1

    # ---- 阶段 C：冲刺 ----
    def run_path(self):
        dist = self.flood(self.goals)
        i, j = self.start
        if dist[i][j] >= INF:
            return []
        out = []
        guard = 0
        while dist[i][j] != 0:
            guard += 1
            if guard > 4000:
                return out
            cur = dist[i][j]
            for d in CW:
                if self.walls[i][j] & d:
                    continue
                ni, nj = i + DIRS[d][0], j + DIRS[d][1]
                if self.inside(ni, nj) and dist[ni][nj] == cur - 1:
                    out.append(d)
                    i, j = ni, nj
                    break
            else:
                return out
        return out

    def drive_home(self):
        self.walk([self.start])


# ---------------------------------------------------------------- 主流程
def main():
    t_start = time.time()
    rclpy.init(args=None)
    cfg = 'config.ini' if os.path.exists('config.ini') else None
    drive = Drive(cfg)

    if not drive.spin_in_thread():
        print('[错误] 未收到 /odom 或 /scan')
        return
    drive.stop()
    drive.calibrate_origin()
    print('[启动] 起点格已标定 origin=(%.3f, %.3f) 初始朝向=%d'
          % (drive.x0, drive.y0, drive.heading_from_yaw()))

    solver = Solver(drive)

    print('[阶段A] 探索')
    t0 = time.time()
    solver.explore()
    print('[阶段A] 结束：移动 %d 步，访问 %d 格，用时 %.1fs'
          % (solver.n_moves, len(solver.explored), time.time() - t0))

    print('[阶段B] 校验')
    t0 = time.time()
    solver.verify()
    print('[阶段B] 结束：用时 %.1fs' % (time.time() - t0))
    if not solver.path_fully_known():
        print('[阶段B] 路径仍不完整，重新建图后再次校验')
        solver.remap()
        solver.verify()

    path = solver.run_path()
    print('[阶段C] 冲刺路径 %d 格' % len(path))

    solver.drive_home()
    drive.turn_to_dir(N)
    print('[阶段C] 已回到起点，开始冲刺（速度 %.2f m/s）' % RUN_SPEED)
    drive.speed_x = RUN_SPEED
    t_run = time.time()

    for d in path:
        if not rclpy.ok():
            break
        cur = drive.heading_from_yaw()          # 每步从里程计重新同步朝向
        if cur == d:
            drive.move_step(FRONT, cur)
        else:
            rel = RIGHT if turn_right(cur) == d else (BACK if OPP[cur] == d else LEFT)
            drive.move_step(rel, cur)

    drive.stop()
    print('[完成] 到达目标：冲刺用时 %.1fs，全程 %.1fs'
          % (time.time() - t_run, time.time() - t_start))
    rclpy.shutdown()


def _safe_main():
    try:
        main()
    except Exception as exc:
        print('[异常]', exc)
    finally:
        try:
            rclpy.try_shutdown()
        except Exception:
            pass
        sys.stdout.flush()
        # 直接退出，避免 spin 线程与解释器析构竞争导致 "terminate called"
        os._exit(0)


if __name__ == '__main__':
    _safe_main()
