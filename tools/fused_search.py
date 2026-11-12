"""融合算法：mms 探索偏置 + Bulebule 收工准则 + 冲刺前校验回合。

设计动机（见 compare_eval 的结论）：
  * Bulebule 式"纯下降 + 反复回补探索"能保证冲刺最短，但探索代价高（maze1 318 格）；
  * mms 式"探索偏置 + 首次到达即停"探索便宜（107 格），但冲刺路线是运气路（maze2 走 99 格）；
  * 两者在噪声下都会因"记忆里的幽灵通道"产出穿墙路线。

本实现把三者拼起来：
  阶段 A 探索：用 mms 的探索偏置走向目标；到达目标后**不立即收工**，
              而是检查"起点→目标"的已知最短路是否所有格子都已访问；
              没有则把路径上第一个未访问格作为临时目标，用纯梯度下降走过去。
              直到已知最短路全程已知 —— 这样冲刺路线必然最优。
  阶段 B 校验：回到起点，沿已知最短路**逐格重新扫一遍墙**。
              只要发现与记忆不符，就更新并重新洪水填充，再查一遍（最多 N 轮）。
              这一步专门消除传感器误判造成的幽灵通道。
  阶段 C 冲刺：执行最终的（已校验的）最短路。

只依赖 robot.read_walls() / robot.move_step()，离线与 Gazebo 通用。
"""
from collections import deque

from maze_model import N, E, S, W, DIRS, OPPOSITE
from relocalize import Relocalizer

FRONT, LEFT, RIGHT, BACK = 0, 1, 2, 3
CW = [N, E, S, W]
INF = 10 ** 6


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


class PhaseStats:
    def __init__(self):
        self.moves = 0
        self.turn90 = 0
        self.turn180 = 0
        self.sense = 0
        self.corrections = 0        # 重新扫描时发现的墙壁修正次数

    def add(self, rel):
        if rel == FRONT:
            self.moves += 1
        elif rel == BACK:
            self.turn180 += 1
            self.moves += 1
        else:
            self.turn90 += 1
            self.moves += 1


class FusedSearch:
    def __init__(self, robot, size=16, start=(0, 0), heading=N, goals=None,
                 verbose=False, relocalize=False, confirm=2, verify=True):
        self.robot = robot
        self.size = size
        self.start = start
        self.heading0 = heading
        self.goals = list(goals or [(7, 7), (7, 8), (8, 7), (8, 8)])
        self.goal_set = set(self.goals)
        self.verbose = verbose

        # 只预置外边界，其余靠探测
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

        self.trace = []                       # 探索阶段走过的格子序列
        self.repeat = 1                       # 感知重复采样次数（重映射时提高到 3）
        self.reloc = Relocalizer(size, confirm=confirm) if relocalize else None
        self.do_verify = verify
        self.stats_explore = PhaseStats()
        self.stats_verify = PhaseStats()
        self.stats = self.stats_explore      # 外部习惯用 .stats 看探索
        self.explore_rounds = 0
        self.verify_rounds_used = 0

    # ---------------- 基础 ----------------
    def inside(self, i, j):
        return 0 <= i < self.size and 0 <= j < self.size

    def _place_wall(self, i, j, d):
        self.walls[i][j] |= d
        di, dj = DIRS[d]
        ni, nj = i + di, j + dj
        if self.inside(ni, nj):
            self.walls[ni][nj] |= OPPOSITE[d]

    def _clear_wall(self, i, j, d):
        """移除一面内部墙（双向），外边界不动。用于纠正假"有墙"。"""
        di, dj = DIRS[d]
        ni, nj = i + di, j + dj
        if not self.inside(ni, nj):
            return False
        if not (self.walls[i][j] & d):
            return False
        self.walls[i][j] &= ~d
        self.walls[ni][nj] &= ~OPPOSITE[d]
        return True

    def flood(self, targets=None):
        targets = targets or self.goals
        dist = [[INF] * self.size for _ in range(self.size)]
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

    def sense(self, stats=None, correct=False, repeat=1):
        """读三面墙。

        correct=False：只增不减（探索阶段）。
        correct=True ：同时清掉与传感器矛盾的记忆墙壁（校验阶段），
                       用于消除假"有墙"造成的死路。
        返回是否有任何墙壁记忆被改变（用于决定是否重新洪水）。
        """
        if repeat > 1:
            # 多次采样取多数，把单次误判概率降到 ~p^repeat
            votes = [self.robot.read_walls() for _ in range(repeat)]
            wl = sum(v[0] for v in votes) * 2 > repeat
            wf = sum(v[1] for v in votes) * 2 > repeat
            wr = sum(v[2] for v in votes) * 2 > repeat
        else:
            wl, wf, wr = self.robot.read_walls()
        (stats or self.stats_explore).sense += 1
        changed = False
        i, j = self.pos
        for has, rel in ((wf, FRONT), (wl, LEFT), (wr, RIGHT)):
            d = rel_to_abs(self.heading, rel)
            if has:
                if not (self.walls[i][j] & d):
                    self._place_wall(i, j, d)
                    changed = True
            elif correct:
                if self._clear_wall(i, j, d):
                    changed = True
        self.explored.add(self.pos)
        # 定位重估：用刚扫到的三面墙在已建地图里重新确定自己在哪
        if self.reloc is not None:
            new_pos, new_heading = self.reloc.update(
                self.walls, self.explored, self.pos, self.heading,
                (wl, wf, wr))
            if new_pos != self.pos or new_heading != self.heading:
                if self.verbose:
                    print(f"[重定位] {self.pos}/{self.heading} -> {new_pos}/{new_heading}")
                self.pos, self.heading = new_pos, new_heading
                self.explored.add(self.pos)
        return changed

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

    def biased_step(self):
        """mms 探索偏置：优先未访问且距离不增的相邻格；否则纯梯度下降。"""
        cur = self.dist[self.pos[0]][self.pos[1]]
        best, best_d = None, INF
        found_unexplored = False
        for rel in (FRONT, LEFT, RIGHT, BACK):
            if not self._can_move(rel):
                continue
            d = self._dist_of(rel)
            unexplored = self._next_pos(rel) not in self.explored
            if unexplored and d <= cur and (not found_unexplored or d < best_d):
                best_d, best, found_unexplored = d, rel, True
            elif not found_unexplored and d < best_d:
                best_d, best = d, rel
        return best

    def descent_step(self):
        """纯梯度下降（用于有明确目标的移动）。"""
        cur = self.dist[self.pos[0]][self.pos[1]]
        for rel in (FRONT, LEFT, RIGHT, BACK):
            if not self._can_move(rel):
                continue
            d = self._dist_of(rel)
            if d == cur - 1:
                return rel
        # 兜底：任意更小的邻居
        best, best_d = None, INF
        for rel in (FRONT, LEFT, RIGHT, BACK):
            if self._can_move(rel):
                d = self._dist_of(rel)
                if d < best_d:
                    best_d, best = d, rel
        return best

    def _apply_step(self, rel):
        """与其它算法同名，便于外部用里程计覆盖为真实定位。"""
        self._apply(rel)

    def _apply(self, rel):
        d = rel_to_abs(self.heading, rel)
        di, dj = DIRS[d]
        self.pos = (self.pos[0] + di, self.pos[1] + dj)
        self.heading = d

    def _step_once(self, rel, stats):
        self.robot.move_step(rel, self.heading)
        stats.add(rel)
        self._apply_step(rel)
        if stats is self.stats_explore:
            self.trace.append(self.pos)

    def _dir_to(self, target):
        """若 target 与当前位置相邻，返回绝对方向。"""
        for d, (di, dj) in DIRS.items():
            if (self.pos[0] + di, self.pos[1] + dj) == target:
                return d
        return None

    def rescan_trace(self):
        """沿探索轨迹逐格复查并双向纠错：所有记录过的墙壁都会被核对一次。"""
        cells = []
        for c in self.trace:
            if not cells or cells[-1] != c:
                cells.append(c)
        n = 0
        for c in cells:
            if c != self.pos:
                d = self._dir_to(c)
                if d is None:
                    continue
                rel = (0 if d == self.heading else
                       1 if d == turn_left(self.heading) else
                       2 if d == OPPOSITE[self.heading] else 3)
                self._step_once(rel, self.stats_verify)
            if self.sense(self.stats_verify, correct=True):
                self.flood(self.goals)
            n += 1
        if self.verbose:
            print(f"[校验] 轨迹复查 {n} 格")

    # ---------------- 行走原语 ----------------
    def _walk(self, targets, stats, bias=False, force_sense=False,
              max_steps=3000, target_set=None, correct=False, repeat=1):
        """走到 targets 之一。bias=True 用 mms 探索偏置，否则纯下降。"""
        target_set = target_set or set(targets)
        self.flood(targets)
        steps = 0
        while steps < max_steps:
            need = force_sense or correct or (self.pos not in self.explored)
            if need:
                if self.sense(stats, correct=correct, repeat=repeat):
                    self.flood(targets)
            if self.pos in target_set:
                return True
            rel = self.biased_step() if bias else self.descent_step()
            if rel is None:
                return False
            self._step_once(rel, stats)
            steps += 1
        return False

    # ---------------- 已知最短路 ----------------
    def known_path(self, start=None, targets=None):
        """在已知墙壁上求最短路，返回格子序列（不含起点）。"""
        start = start or self.start
        targets = targets or self.goals
        dist = self.flood(targets)
        i, j = start
        if dist[i][j] >= INF:
            return None
        path = []
        guard = 0
        while dist[i][j] != 0:
            guard += 1
            if guard > 4000:
                return None
            cur = dist[i][j]
            moved = False
            for d in CW:
                if self.walls[i][j] & d:
                    continue
                di, dj = DIRS[d]
                ni, nj = i + di, j + dj
                if self.inside(ni, nj) and dist[ni][nj] == cur - 1:
                    path.append((ni, nj))
                    i, j = ni, nj
                    moved = True
                    break
            if not moved:
                return None
        return path

    def path_fully_known(self):
        p = self.known_path()
        return p is not None and all(c in self.explored for c in p)

    # ---------------- 阶段 A：探索 ----------------
    def explore(self, max_rounds=60):
        self.flood(self.goals)
        for _ in range(max_rounds):
            self.explore_rounds += 1
            self._walk(self.goals, self.stats_explore, bias=True, repeat=self.repeat)
            if self.pos in self.goal_set and self.path_fully_known():
                break
            # 目标路径上有未访问格 -> 作为临时目标专程去一趟
            p = self.known_path()
            if p is None:
                break
            target = next((c for c in p if c not in self.explored), None)
            if target is None:
                break
            self._walk([target], self.stats_explore, bias=False, repeat=self.repeat)
            if self.verbose:
                print(f"[探索] 第 {self.explore_rounds} 轮：补探 {target}，已访问 {len(self.explored)} 格")
        self.stats = self.stats_explore
        return self.stats_explore

    # ---------------- 阶段 B：校验 ----------------
    def _wall_count(self):
        return sum(v.bit_count() for row in self.walls for v in row)

    def scan_full(self):
        """在当前格原地转一圈，四个朝向各扫一次（用于彻底核对一格）。"""
        changed = False
        h0 = self.heading
        for k in range(4):
            if self.sense(self.stats_verify, correct=True):
                changed = True
            if k < 3:
                self._turn_rel(RIGHT)
        while self.heading != h0:
            self._turn_rel(RIGHT)
        return changed

    def _turn_rel(self, rel):
        """原地转向（不前进），计入校验阶段统计。"""
        d = rel_to_abs(self.heading, rel)
        fn = getattr(self.robot, "turn_only", None)
        if fn is not None:
            fn(d)
        if rel == BACK:
            self.stats_verify.turn180 += 1
        else:
            self.stats_verify.turn90 += 1
        self.heading = d

    def repair_candidates(self, limit=8):
        """目标不可达时，按"去除后路径最短"排序的可疑墙列表 [(cell, dir), ...]。"""
        self.flood(self.goals)
        d_goal = self.dist
        dist_home = [[INF] * self.size for _ in range(self.size)]
        q = deque([self.start])
        dist_home[self.start[0]][self.start[1]] = 0
        while q:
            i, j = q.popleft()
            for d, (di, dj) in DIRS.items():
                if self.walls[i][j] & d:
                    continue
                ni, nj = i + di, j + dj
                if self.inside(ni, nj) and dist_home[ni][nj] == INF:
                    dist_home[ni][nj] = dist_home[i][j] + 1
                    q.append((ni, nj))
        cands = []
        for i in range(self.size):
            for j in range(self.size):
                if dist_home[i][j] >= INF:
                    continue
                for d, (di, dj) in DIRS.items():
                    if not (self.walls[i][j] & d):
                        continue
                    ni, nj = i + di, j + dj
                    if not self.inside(ni, nj) or d_goal[ni][nj] >= INF:
                        continue
                    cands.append((dist_home[i][j] + 1 + d_goal[ni][nj], i, j, d))
        cands.sort()
        return [(i, j, d) for _, i, j, d in cands[:limit]]

    def _walk_path_sensing(self):
        """从起点沿已知最短路走到目标，逐格只增不减地感知；返回新增墙壁数。"""
        before = self._wall_count()
        self._walk(self.goals, self.stats_verify, bias=False,
                   force_sense=True, correct=False, repeat=3)
        return self._wall_count() - before

    def verify(self, max_rounds=8):
        """冲刺前校验：只增不减地重扫路径；不可达时定点清除可疑墙并实地验证。

        原则：绝不因为"这次没扫到"就删墙（传感器会漏检），
        只有在"确认为不可达"时才对可疑墙做一次清除，并立刻实地走一遍验证，
        若被重新观测到就自动恢复（_place_wall）。
        """
        cleared = set()
        for r in range(1, max_rounds + 1):
            self.verify_rounds_used = r
            if self.pos != self.start:
                self._walk([self.start], self.stats_verify, bias=False)
            if self.known_path() is not None:
                added = self._walk_path_sensing()
                if added == 0 and self.path_fully_known():
                    if self.verbose:
                        print(f"[校验] 第 {r} 轮通过（路径已实地复核）")
                    return True
                if self.verbose:
                    print(f"[校验] 第 {r} 轮：路径上新发现 {added} 面墙，重算")
                continue
            # 目标不可达：逐个尝试清除可疑墙并实地验证
            progressed = False
            for (i, j, d) in self.repair_candidates():
                if (i, j, d) in cleared:
                    continue
                cleared.add((i, j, d))
                self._clear_wall(i, j, d)
                if self.known_path() is None:
                    self._place_wall(i, j, d)      # 清了也没用，立刻还原
                    continue
                if self.verbose:
                    print(f"[校验] 第 {r} 轮：假设 ({i},{j}) 方向{d} 无墙，实地验证")
                self._walk_path_sensing()
                progressed = True
                if self.pos in self.goal_set and self.path_fully_known():
                    return True
            if not progressed:
                break
        return self.path_fully_known()

    def remap(self):
        """地图不一致时的兜底：清空墙壁记忆，用重复采样重新探索一遍。"""
        for i in range(self.size):
            for j in range(self.size):
                self.walls[i][j] = 0
        for i in range(self.size):
            self.walls[i][0] |= S
            self.walls[i][self.size - 1] |= N
            self.walls[0][i] |= W
            self.walls[self.size - 1][i] |= E
        self.explored.clear()
        self.trace.clear()
        self.repeat = 3
        if self.verbose:
            print("[兜底] 地图自相矛盾，清空后重新建图（3 次采样取多数）")
        self._walk([self.start], self.stats_verify, bias=False, repeat=3)
        self.pos = self.start
        self.explore()
        self.repeat = 1

    # ---------------- 阶段 C：冲刺序列 ----------------
    def build_run_path(self):
        """已知最短路 -> (next_i, next_j, dir) 列表。"""
        dist = self.flood(self.goals)
        i, j = self.start
        if dist[i][j] >= INF:
            return []
        path = []
        guard = 0
        while dist[i][j] != 0:
            guard += 1
            if guard > 4000:
                return path
            cur = dist[i][j]
            for d in CW:
                if self.walls[i][j] & d:
                    continue
                di, dj = DIRS[d]
                ni, nj = i + di, j + dj
                if self.inside(ni, nj) and dist[ni][nj] == cur - 1:
                    path.append((ni, nj, d))
                    i, j = ni, nj
                    break
            else:
                return path
        return path

    def solve(self, max_cycles=6):
        """完整流程：探索 → 校验 →（必要时补探/重建）→ 返回冲刺路径。

        收工条件：已知迷宫里目标可达 **且** 起点→目标最短路全程都是已访问格。
        """
        self.explore()
        if not self.do_verify:
            return self.build_run_path()
        for _ in range(max_cycles):
            ok = self.verify()
            if ok and self.path_fully_known():
                return self.build_run_path()
            self.explore()                      # 补探新暴露出来的未访问格
        self.remap()                            # 兜底：清空重建
        self.verify()
        if not self.path_fully_known():
            self.explore()
        return self.build_run_path()
