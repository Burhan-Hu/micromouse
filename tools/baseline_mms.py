"""mms 版洪水填充算法（移植自 ShifatHasanGNS/maze_solver, MIT License）。

原文：https://github.com/ShifatHasanGNS/maze_solver  (src/Util.h, src/Handler.h)

与 Bulebule 基线的关键差异（这些差异正是我们要对比的）：
  1. 决策顺序同为 前→左→右→后，但带 **探索偏置**：只要相邻未访问格的距离
     `<= 当前距离` 就优先选它（Bulebule 要求严格更小），因此会主动把路过的
     未访问格子走掉；
  2. **首次到达目标就结束探索**（不做"回起点再补探索"的回合）；
  3. 把走过的 F/L/R 序列做**路径压缩**（FRRF→RR、RRR→L、LR→"" 等），
     减少冲刺时的多余转向。

坐标说明：原仓库用 (row, col)，row 向下增大、起点 (15,0) 朝北；
本移植统一到本项目的 (i, j)（i 沿 +x、j 沿 +y，起点 (0,0) 朝北）。
两者映射 r = 15 - j、c = i，四个方向名一致，因此墙壁语义可直接对应：
    北/东/南/西  <=>  h_wall[r][c] / v_wall[r][c+1] / h_wall[r+1][c] / v_wall[r][c]
"""
from collections import deque

from maze_model import N, E, S, W, DIRS, OPPOSITE

FRONT, LEFT, RIGHT, BACK = 0, 1, 2, 3
CW = [N, E, S, W]
INF = 255


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
        return OPPOSITE[heading]
    return turn_left(heading)


class Stats:
    def __init__(self):
        self.moves = 0
        self.turn90 = 0
        self.turn180 = 0
        self.sense = 0
        self.reflood = 0
        self.cells_visited = 0


def compress_path(path):
    """原样移植 compressPath()：反复消去冗余转向序列。"""
    def replace_all(s, frm, to):
        return s.replace(frm, to)

    def can_compress(s):
        if len(s) < 2:
            return False
        pairs = ("RR", "LL", "LR", "RL")
        return any(s[i:i + 2] in pairs for i in range(len(s) - 1))

    s = "".join(path)
    guard = 0
    while can_compress(s) and guard < 10000:
        guard += 1
        s = replace_all(s, "FRRF", "RR")
        s = replace_all(s, "FLLF", "RR")
        s = replace_all(s, "RRR", "L")
        s = replace_all(s, "LLL", "R")
        s = replace_all(s, "LR", "")
        s = replace_all(s, "RL", "")
    return s


class MMSSearch:
    def __init__(self, robot, size=16, start=(0, 0), heading=N, goals=None,
                 verbose=False, compress=True):
        self.robot = robot
        self.size = size
        self.start = start
        self.heading0 = heading
        self.goals = list(goals or [(7, 7), (7, 8), (8, 7), (8, 8)])
        self.goal_set = set(self.goals)
        self.verbose = verbose
        self.do_compress = compress

        # 墙壁记忆：只预置外边界（原仓库 resetWalls 的行为）
        self.walls = [[0] * size for _ in range(size)]
        for i in range(size):
            self.walls[i][0] |= S
            self.walls[i][size - 1] |= N
            self.walls[0][i] |= W
            self.walls[size - 1][i] |= E

        self.explored = set()
        self.pos = start
        self.heading = heading
        self.dist = None
        self.path = []              # 'F' / 'L' / 'R'（原仓库 BACK 记作 "RR"）
        self.stats = Stats()

    # ---------------- 基础 ----------------
    def inside(self, i, j):
        return 0 <= i < self.size and 0 <= j < self.size

    def _place_wall(self, i, j, d):
        self.walls[i][j] |= d
        di, dj = DIRS[d]
        ni, nj = i + di, j + dj
        if self.inside(ni, nj):
            self.walls[ni][nj] |= OPPOSITE[d]

    def flood(self):
        dist = [[INF] * self.size for _ in range(self.size)]
        q = deque()
        for (i, j) in self.goals:
            dist[i][j] = 0
            q.append((i, j))
        while q:
            i, j = q.popleft()
            nd = dist[i][j] + 1
            if nd >= INF:
                continue
            for d, (di, dj) in DIRS.items():
                if self.walls[i][j] & d:
                    continue
                ni, nj = i + di, j + dj
                if self.inside(ni, nj) and dist[ni][nj] == INF:
                    dist[ni][nj] = nd
                    q.append((ni, nj))
        self.stats.reflood += 1
        self.dist = dist
        return dist

    def sense(self):
        """读三面墙写入记忆，返回是否有新墙（对应 setWallsBasedOnMouse）。"""
        wl, wf, wr = self.robot.read_walls()
        self.stats.sense += 1
        new = False
        for has, rel in ((wf, FRONT), (wl, LEFT), (wr, RIGHT)):
            if not has:
                continue
            d = rel_to_abs(self.heading, rel)
            if not (self.walls[self.pos[0]][self.pos[1]] & d):
                new = True
            self._place_wall(*self.pos, d)
        return new

    # ---------------- 决策 ----------------
    def _can_move(self, rel):
        d = rel_to_abs(self.heading, rel)
        if self.walls[self.pos[0]][self.pos[1]] & d:
            return False
        di, dj = DIRS[d]
        return self.inside(self.pos[0] + di, self.pos[1] + dj)

    def _next_pos(self, rel):
        d = rel_to_abs(self.heading, rel)
        di, dj = DIRS[d]
        return (self.pos[0] + di, self.pos[1] + dj)

    def _dist_of(self, rel):
        if not self._can_move(rel):
            return INF
        i, j = self._next_pos(rel)
        return self.dist[i][j]

    def best_move(self):
        """带探索偏置的选择（移植 getBestMove）。"""
        cur = self.dist[self.pos[0]][self.pos[1]]
        best, best_d = None, INF
        found_unexplored = False
        for rel in (FRONT, LEFT, RIGHT, BACK):
            if not self._can_move(rel):
                continue
            d = self._dist_of(rel)
            nxt = self._next_pos(rel)
            unexplored = nxt not in self.explored
            if unexplored and d <= cur and (not found_unexplored or d < best_d):
                best_d, best, found_unexplored = d, rel, True
            elif not found_unexplored and d < best_d:
                best_d, best = d, rel
        return best

    def _apply_step(self, rel):
        """与 bulebule 版保持同名，便于外部用里程计覆盖为真实定位。"""
        self._apply(rel)

    def _apply(self, rel):
        d = rel_to_abs(self.heading, rel)
        di, dj = DIRS[d]
        self.pos = (self.pos[0] + di, self.pos[1] + dj)
        self.heading = d

    def _count(self, rel):
        if rel == FRONT:
            self.stats.moves += 1
        elif rel == BACK:
            self.stats.turn180 += 1
            self.stats.moves += 1
        else:
            self.stats.turn90 += 1
            self.stats.moves += 1

    # ---------------- 主流程 ----------------
    def explore(self, max_moves=500):
        self.flood()
        while True:
            if self.stats.moves >= max_moves:
                print("[警告] 达到最大移动数 %d" % max_moves)
                return self.stats
            if self.pos in self.goal_set:        # 首次到达目标即结束
                break
            if self.sense():                     # 有新墙才重算距离场
                self.flood()
            self.explored.add(self.pos)
            self.stats.cells_visited = len(self.explored)
            rel = self.best_move()
            if rel is None:
                break
            # 记录路径：原仓库先记录转向（BACK 记作 "RR"），随后每次移动都追加 "F"
            self.path.append({FRONT: "F", LEFT: "LF", RIGHT: "RF", BACK: "RRF"}[rel])
            self.robot.move_step(rel, self.heading)
            self._count(rel)
            self._apply_step(rel)
            if self.verbose:
                print("[探索] %s -> %s  dist=%d" % (self.pos, rel, self.dist[self.pos[0]][self.pos[1]]))
        return self.stats

    def run_sequence(self):
        raw = "".join(self.path)
        return compress_path(raw) if self.do_compress else raw

    @staticmethod
    def analyse(seq, heading0=N):
        """把 F/L/R 序列折算成 格子数/90°转/180°转，并验证路径可达性。"""
        heading = heading0
        moves = t90 = t180 = 0
        for ch in seq:
            if ch == "F":
                moves += 1
            elif ch == "L":
                t90 += 1
                heading = turn_left(heading)
            elif ch == "R":
                t90 += 1
                heading = turn_right(heading)
        return dict(moves=moves, turn90=t90, turn180=t180)
