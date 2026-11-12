"""16x16 迷宫模型：从 .world 提取真实墙壁，或从 .TQD 文本解析。

坐标约定（与 contest.py / config.ini 保持一致）：
    单元格 (i, j)，i 沿 +x，j 沿 +y；r=0 为最南（y=-1.35），j=15 为最北。
    NORTH = +y, EAST = +x, SOUTH = -y, WEST = -x
起点 (0, 0)，车头朝 NORTH。

几何（来自 world/maze*.world）：
    单元格边长 0.18 m，墙厚 0.012 m，墙高 0.05 m
    单元格中心 x = -1.35 + 0.18*i, y = -1.35 + 0.18*j
"""
import math
import xml.etree.ElementTree as ET
from collections import deque

N, E, S, W = 1, 2, 4, 8
DIRS = {N: (0, 1), E: (1, 0), S: (0, -1), W: (-1, 0)}
OPPOSITE = {N: S, E: W, S: N, W: E}
CELL = 0.18
CENTER0 = -1.35
SIZE = 16


class Maze:
    """16x16 迷宫，walls[i][j] 为 4 位掩码（N/E/S/W）。"""

    def __init__(self, size=SIZE):
        self.size = size
        self.walls = [[0] * size for _ in range(size)]
        # 外边界默认全封闭（IEEE 规则）
        for i in range(size):
            for j in range(size):
                if j == size - 1:
                    self.add(i, j, N)
                if j == 0:
                    self.add(i, j, S)
                if i == size - 1:
                    self.add(i, j, E)
                if i == 0:
                    self.add(i, j, W)

    def inside(self, i, j):
        return 0 <= i < self.size and 0 <= j < self.size

    def add(self, i, j, d, both=True):
        self.walls[i][j] |= d
        di, dj = DIRS[d]
        ni, nj = i + di, j + dj
        if both and self.inside(ni, nj):
            self.walls[ni][nj] |= OPPOSITE[d]

    def has(self, i, j, d):
        return bool(self.walls[i][j] & d)

    def copy(self):
        m = Maze.__new__(Maze)
        m.size = self.size
        m.walls = [row[:] for row in self.walls]
        return m

    # ---------- 距离场 ----------
    def distances(self, targets):
        """BFS 洪水填充：返回 16x16 距离场（不可达为 None）。"""
        dist = [[None] * self.size for _ in range(self.size)]
        q = deque()
        for (i, j) in targets:
            if self.inside(i, j):
                dist[i][j] = 0
                q.append((i, j))
        while q:
            i, j = q.popleft()
            d = dist[i][j] + 1
            for wd, (di, dj) in DIRS.items():
                if self.has(i, j, wd):
                    continue
                ni, nj = i + di, j + dj
                if not self.inside(ni, nj) or dist[ni][nj] is not None:
                    continue
                dist[ni][nj] = d
                q.append((ni, nj))
        return dist

    def path(self, start, targets):
        """沿距离场下降得到最短路（返回 [(i,j,dir), ...]）。"""
        dist = self.distances(targets)
        if dist[start[0]][start[1]] is None:
            return None
        i, j = start
        steps = []
        while dist[i][j] != 0:
            cur = dist[i][j]
            for wd, (di, dj) in DIRS.items():
                if self.has(i, j, wd):
                    continue
                ni, nj = i + di, j + dj
                if self.inside(ni, nj) and dist[ni][nj] is not None and dist[ni][nj] == cur - 1:
                    steps.append((ni, nj, wd))
                    i, j = ni, nj
                    break
            else:
                return None
        return steps

    def bfs_path_len(self, start, targets):
        p = self.path(start, targets)
        return None if p is None else len(p)

    # ---------- 拓扑统计 ----------
    def open_edges(self):
        n = 0
        for i in range(self.size):
            for j in range(self.size):
                if not self.has(i, j, E) and i + 1 < self.size:
                    n += 1
                if not self.has(i, j, N) and j + 1 < self.size:
                    n += 1
        return n

    def total_edges(self):
        # 邻居边总数（含外边界）
        return 2 * self.size * (self.size - 1) + 2 * self.size

    # ---------- 渲染 ----------
    def to_ascii(self):
        rows = []
        for j in reversed(range(self.size)):
            top = ""
            for i in range(self.size):
                top += "+" + ("---" if self.has(i, j, N) else "   ")
            rows.append(top + "+")
            mid = ""
            for i in range(self.size):
                mid += ("|" if self.has(i, j, W) else " ") + "   "
            rows.append(mid + ("|" if self.has(self.size - 1, j, E) else " "))
        bot = ""
        for i in range(self.size):
            bot += "+" + ("---" if self.has(i, 0, S) else "   ")
        rows.append(bot + "+")
        return "\n".join(rows)


# ============================================================ .world
def from_world(path, size=SIZE, verbose=False):
    """从 Gazebo .world 的 my_maze 模型 collision 盒子反推墙壁。

    水平墙（size.x > size.y）位于单元格中心 x、单元格边界 y；
    垂直墙反之。
    """
    root = ET.parse(path).getroot()
    maze = Maze(size)
    # 清空默认边界，完全依据几何重建
    maze.walls = [[0] * size for _ in range(size)]

    model = None
    for m in root.iter("model"):
        if m.get("name") == "my_maze":
            model = m
            break
    if model is None:
        raise ValueError("no my_maze model in %s" % path)

    def cell_index(v):
        return (v - CENTER0) / CELL

    n_walls = 0
    for link in model.findall("link"):
        for col in link.findall("collision"):
            pose = col.find("pose")
            box = col.find("geometry/box/size")
            if pose is None or box is None:
                continue
            vals = [float(v) for v in pose.text.split()]
            px, py = vals[0], vals[1]
            yaw = vals[5] if len(vals) > 5 else 0.0
            sx, sy = [float(v) for v in box.text.split()[:2]]
            if abs(sx - sy) < 1e-6:          # 地板
                continue
            n_walls += 1
            # 墙盒统一为 0.192 x 0.012，用偏航角区分横竖
            horizontal = abs(math.sin(yaw)) < 0.5 and sx > sy
            if horizontal:                    # 水平墙：y 为边界
                i = round(cell_index(px))
                k = round(cell_index(py) + 0.5)   # 边界序号 0..16
                if 0 <= i < size and 0 <= k <= size:
                    if k < size:
                        maze.add(i, k, S, both=False)
                    if k - 1 >= 0:
                        maze.add(i, k - 1, N, both=False)
            else:                             # 垂直墙：x 为边界
                j = round(cell_index(py))
                k = round(cell_index(px) + 0.5)
                if 0 <= j < size and 0 <= k <= size:
                    if k < size:
                        maze.add(k, j, W, both=False)
                    if k - 1 >= 0:
                        maze.add(k - 1, j, E, both=False)
    if verbose:
        print("walls parsed:", n_walls)
    return maze


# ============================================================ .TQD
def from_tqd(path, size=SIZE):
    """解析 16 行 x 33 字符的 .TQD 迷宫文本。

    经与 world/maze*.world 几何逐一比对验证（3 个迷宫 0 误差），格式为：
        '|' 位于偶数列 (0,2,...,2*size) -> 该格 W 墙（末列是第 size-1 格的 E 墙）
        '_' 位于奇数列 (1,3,...,2*size-1) -> 该格 S 墙（即画在格子下方）
        第 0 行为最北一行 (j = size-1)
    """
    lines = [l.rstrip("\n") for l in open(path, encoding="utf-8") if l.strip()]
    if len(lines) != size:
        raise ValueError("%s: expect %d rows, got %d" % (path, size, len(lines)))

    maze = Maze(size)
    maze.walls = [[0] * size for _ in range(size)]

    for r, line in enumerate(lines):
        j = size - 1 - r
        if len(line) < 2 * size + 1:
            raise ValueError("%s row %d too short" % (path, r))
        for i in range(size):
            if line[2 * i] == "|":
                maze.walls[i][j] |= W
            if line[2 * i + 1] == "_":
                maze.walls[i][j] |= S
        if line[2 * size] == "|":
            maze.walls[size - 1][j] |= E

    # 外边界（IEEE 规则：整个迷宫封闭）
    for i in range(size):
        maze.walls[i][0] |= S
        maze.walls[i][size - 1] |= N
        maze.walls[0][i] |= W
        maze.walls[size - 1][i] |= E

    # 对称化，保证相邻格墙壁一致
    for i in range(size):
        for j in range(size):
            for d, (di, dj) in DIRS.items():
                ni, nj = i + di, j + dj
                if maze.inside(ni, nj):
                    if maze.has(i, j, d) or maze.has(ni, nj, OPPOSITE[d]):
                        maze.walls[i][j] |= d
                        maze.walls[ni][nj] |= OPPOSITE[d]
    return maze
