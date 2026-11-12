"""定位重估：用"局部墙壁签名"把里程计漂移纠正回真实格子。

原理
    真实的里程计会漏计/多计格子（打滑），一旦发生，往后所有墙壁都会被记到
    错误的坐标上，整张地图就废了。但传感器本身没有漂移：
    在任何一个位置，扫描到的三面墙（左/前/右）就是真实墙壁的局部指纹。

做法
    把"当前三面墙 + 朝向"在**已建好的地图**里做一次全局匹配：
        对每个候选格 c 和每个朝向 h，
            期望 = (map[c] 有左墙, 有前墙, 有右墙)
            得分 = 与实测一致的面数 - 与实测矛盾的面数
    取最高分（并列时取离当前位置最近的），作为"我其实在哪"。
    连续 N 次不一致才纠正，避免单次误判造成抖动。

只有"已探测过的格子"才参与匹配，避免把未知区域当成空通道。
"""

N, E, S, W = 1, 2, 4, 8
CW = [N, E, S, W]
OPP = {N: S, E: W, S: N, W: E}
DIRS = {N: (0, 1), E: (1, 0), S: (0, -1), W: (-1, 0)}


def rel_to_abs(heading, rel):      # rel: 0 前, 1 左, 2 右
    if rel == 0:
        return heading
    if rel == 1:
        return CW[(CW.index(heading) + 3) % 4]
    return CW[(CW.index(heading) + 1) % 4]


def abs_to_world(d):
    return {N: 1.5707963267948966, E: 0.0, S: -1.5707963267948966, W: 3.141592653589793}[d]


class Relocalizer:
    def __init__(self, size=16, confirm=2, min_margin=0):
        self.size = size
        self.confirm = confirm
        self.min_margin = min_margin
        self.pending = 0
        self.corrections = 0
        self.history = []

    def inside(self, i, j):
        return 0 <= i < self.size and 0 <= j < self.size

    def score_at(self, walls, explored, cell, heading, obs):
        """返回 (得分, 是否矛盾)。obs = (左, 前, 右) 的 bool。"""
        i, j = cell
        if cell not in explored:
            return None, False
        score = 0
        contradiction = False
        mask = walls[i][j]
        for rel, seen in zip((1, 0, 2), obs):     # 左 / 前 / 右
            d = rel_to_abs(heading, rel)
            has = bool(mask & d)
            if has == seen:
                score += 1
            else:
                contradiction = True
        return score, contradiction

    def locate(self, walls, explored, est_pos, est_heading, obs):
        """全局匹配，返回 (最佳格, 最佳朝向, 得分差) 或 None。"""
        best = []
        for i in range(self.size):
            for j in range(self.size):
                if (i, j) not in explored:
                    continue
                for h in CW:
                    s, bad = self.score_at(walls, explored, (i, j), h, obs)
                    if s is None:
                        continue
                    # 得分优先；同分则用与估计位置的距离打破平局
                    dist = abs(i - est_pos[0]) + abs(j - est_pos[1])
                    best.append((s, -dist, i, j, h))
        if not best:
            return None
        best.sort(reverse=True)
        s0 = best[0]
        s1 = best[1][0] if len(best) > 1 else -1
        return (s0[2], s0[3]), s0[4], s0[0] - s1

    def update(self, walls, explored, est_pos, est_heading, obs):
        """返回修正后的 (pos, heading)；不纠正则原样返回。"""
        res = self.locate(walls, explored, est_pos, est_heading, obs)
        if res is None:
            return est_pos, est_heading
        found, heading, margin = res
        if found == est_pos and heading == est_heading:
            self.pending = 0
            return est_pos, est_heading
        # 只在"匹配到已访问格、且不是只差朝向"时纠正位置
        self.pending += 1
        if self.pending >= self.confirm:
            self.pending = 0
            if found != est_pos:
                self.corrections += 1
                self.history.append((est_pos, found))
            return found, heading
        return est_pos, est_heading
