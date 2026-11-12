"""Bulebule 风格洪水填充算法的 Python 重实现（仅在算法思想上对齐，代码为独立编写）。

Bulebule 基线（Bulebots/bulebule + mmlib，GPLv3，未复制其代码）的要点：
  1. 4 位墙壁位域 + VISITED 标记的 16x16 迷宫记忆；
  2. 每次移动前用 BFS 洪水填充生成到目标的距离场；
  3. 选择下一步：前 -> 左 -> 右，要求距离"严格更小"，都不满足则后退；
  4. 到达目标后若不在起点，就沿"起点->目标"的已知最短路找出第一个未访问格，
     作为新目标继续探索；直到起点->目标路径上不再有未访问格为止；
  5. 探索结束后把路径编译成 F/L/R 序列做冲刺。

本模块只依赖 maze_model 的坐标约定，通过 RobotInterface 与外部（离线仿真 / Gazebo）解耦。
"""
from collections import deque

from maze_model import N, E, S, W, DIRS, OPPOSITE

# 相对方向: 0=前 1=左 2=右 3=后
FRONT, LEFT, RIGHT, BACK = 0, 1, 2, 3
# 以 NORTH 为 0 的顺时针序号，用于相对方向换算
_CW = [N, E, S, W]          # 顺时针
_TURN_RIGHT = {FRONT: FRONT, LEFT: None, RIGHT: None, BACK: BACK}


def turn_right(d):
    return _CW[(_CW.index(d) + 1) % 4]


def turn_left(d):
    return _CW[(_CW.index(d) + 3) % 4]


def rel_to_abs(heading, rel):
    """相对方向 -> 绝对方向。"""
    if rel == FRONT:
        return heading
    if rel == RIGHT:
        return turn_right(heading)
    if rel == BACK:
        return OPPOSITE[heading]
    return turn_left(heading)


class Stats:
    def __init__(self):
        self.moves = 0            # 前进格数
        self.turn90 = 0           # 90 度转向次数
        self.turn180 = 0          # 180 度转向次数
        self.sense = 0            # 读取墙壁次数
        self.reflood = 0          # 洪水填充次数
        self.cells_visited = 0

    def as_dict(self):
        return dict(moves=self.moves, turn90=self.turn90, turn180=self.turn180,
                    sense=self.sense, reflood=self.reflood,
                    cells_visited=self.cells_visited)


class FloodFillSearch:
    """Bulebule 风格的洪水填充探索 + 冲刺。"""

    def __init__(self, robot, size=16, start=(0, 0), heading=N, goals=None,
                 verbose=False):
        self.robot = robot
        self.size = size
        self.start = start
        self.heading0 = heading
        self.goals = goals or [(7, 7), (7, 8), (8, 7), (8, 8)]

        # 已知墙壁（只含已探测信息），初始只有外边界
        self.known = [[0] * size for _ in range(size)]
        for i in range(size):
            self.known[i][0] |= S
            self.known[i][size - 1] |= N
            self.known[0][i] |= W
            self.known[size - 1][i] |= E
        self.visited = set()

        self.pos = start
        self.heading = heading
        self.dist = None
        self.verbose = verbose
        self.stats = Stats()

    # ---------------- 基础 ----------------
    def inside(self, i, j):
        return 0 <= i < self.size and 0 <= j < self.size

    def has_wall(self, i, j, d):
        return bool(self.known[i][j] & d)

    def _place_wall(self, i, j, d):
        self.known[i][j] |= d
        di, dj = DIRS[d]
        ni, nj = i + di, j + dj
        if self.inside(ni, nj):
            self.known[ni][nj] |= OPPOSITE[d]

    def flood(self, targets):
        """BFS 距离场；不可达为 None。"""
        dist = [[None] * self.size for _ in range(self.size)]
        q = deque()
        for t in targets:
            if self.inside(*t):
                dist[t[0]][t[1]] = 0
                q.append(t)
        while q:
            i, j = q.popleft()
            d0 = dist[i][j] + 1
            for d, (di, dj) in DIRS.items():
                if self.has_wall(i, j, d):
                    continue
                ni, nj = i + di, j + dj
                if self.inside(ni, nj) and dist[ni][nj] is None:
                    dist[ni][nj] = d0
                    q.append((ni, nj))
        self.stats.reflood += 1
        return dist

    def _here(self):
        return self.dist[self.pos[0]][self.pos[1]]

    # ---------------- 感知 ----------------
    def sense(self):
        """读取当前位置的三面墙并写入已知迷宫（对应 Bulebule read_walls/update_walls）。"""
        wl, wf, wr = self.robot.read_walls()   # 相对车头: 左/前/右
        self.stats.sense += 1
        if wf:
            self._place_wall(*self.pos, rel_to_abs(self.heading, FRONT))
        if wl:
            self._place_wall(*self.pos, rel_to_abs(self.heading, LEFT))
        if wr:
            self._place_wall(*self.pos, rel_to_abs(self.heading, RIGHT))
        self.visited.add(self.pos)
        self.stats.cells_visited = len(self.visited)

    # ---------------- 决策 ----------------
    def _dist_at(self, rel):
        d = rel_to_abs(self.heading, rel)
        if self.has_wall(*self.pos, d):
            return None
        di, dj = DIRS[d]
        ni, nj = self.pos[0] + di, self.pos[1] + dj
        if not self.inside(ni, nj):
            return None
        return self.dist[ni][nj]

    def best_step(self):
        """前 -> 左 -> 右（严格下降），否则后退。"""
        cur = self._here()
        for rel in (FRONT, LEFT, RIGHT):
            v = self._dist_at(rel)
            if v is not None and cur is not None and v < cur:
                return rel
        return BACK

    def _apply_step(self, rel):
        d = rel_to_abs(self.heading, rel)
        di, dj = DIRS[d]
        self.pos = (self.pos[0] + di, self.pos[1] + dj)
        self.heading = d

    # ---------------- 主流程 ----------------
    def go_to_target(self, targets):
        """走到距离场为 0 的格子，途中实时补墙并重新洪水填充。"""
        self.dist = self.flood(targets)
        while True:
            cur = self._here()
            if cur is None:                       # 目标不可达
                return False
            if self.pos not in self.visited:
                self.sense()
                self.dist = self.flood(targets)
            if self._here() == 0:
                return True
            rel = self.best_step()
            self.robot.move_step(rel, self.heading)
            self._count_motion(rel)
            self._apply_step(rel)

    def _count_motion(self, rel):
        if rel == FRONT:
            self.stats.moves += 1
        elif rel == BACK:
            self.stats.turn180 += 1
            self.stats.moves += 1
        else:
            self.stats.turn90 += 1
            self.stats.moves += 1

    def path_from_start_to_goal(self):
        """用已知墙壁模拟从起点走到目标，返回途径格序列。"""
        i, j = self.start
        heading = self.heading0
        seq = [(i, j)]
        dist = self.flood(self.goals)
        guard = 0
        while dist[i][j] not in (0, None):
            guard += 1
            if guard > 4 * self.size * self.size:
                break
            cur = dist[i][j]
            nxt = None
            for rel in (FRONT, LEFT, RIGHT, BACK):
                d = rel_to_abs(heading, rel)
                if self.known[i][j] & d:
                    continue
                di, dj = DIRS[d]
                ni, nj = i + di, j + dj
                if self.inside(ni, nj) and dist[ni][nj] is not None and dist[ni][nj] == cur - 1:
                    nxt = (ni, nj, d)
                    break
            if nxt is None:
                break
            i, j, heading = nxt
            seq.append((i, j))
        return seq

    def find_unexplored_cell(self):
        """沿 起点->目标 的已知最短路找第一个未访问格；没有则返回 None。"""
        for cell in self.path_from_start_to_goal():
            if cell not in self.visited:
                return cell
        return None

    def explore(self, max_rounds=1000):
        """探索到"起点->目标路径上无未访问格"为止，结束时回到起点。"""
        self.dist = self.flood(self.goals)
        rounds = 0
        while rounds < max_rounds:
            rounds += 1
            if not self.go_to_target(self.goals):
                break
            if self.verbose:
                print("[探索] 到达目标 %s，已访问 %d 格" % (self.pos, len(self.visited)))
            if self.pos == self.start:
                break
            cell = self.find_unexplored_cell()
            if cell is None:                      # 路径已完全已知 -> 回起点
                self.go_to_target([self.start])
                break
            self.go_to_target([cell])
        return self.stats

    # ---------------- 冲刺 ----------------
    def build_run_path(self):
        """用最终已知墙壁求 起点->目标 最短路（绝对方向序列）。"""
        dist = self.flood(self.goals)
        i, j = self.start
        path = []
        if dist[i][j] is None:          # 已知迷宫里目标不可达（多为墙壁误判）
            print("[警告] 已知迷宫中目标不可达，冲刺路线为空")
            return path
        guard = 0
        while dist[i][j] != 0:
            guard += 1
            if guard > 4 * self.size * self.size:
                return path
            cur = dist[i][j]
            moved = False
            for d in _CW:
                if self.known[i][j] & d:
                    continue
                di, dj = DIRS[d]
                ni, nj = i + di, j + dj
                if self.inside(ni, nj) and dist[ni][nj] == cur - 1:
                    path.append((ni, nj, d))
                    i, j = ni, nj
                    moved = True
                    break
            if not moved:
                break
        return path
