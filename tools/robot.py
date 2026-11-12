"""机器人接口层：离线仿真机器人与 Gazebo(ROS2) 机器人。

算法只依赖两个方法：
    read_walls() -> (left, front, right)   # 相对车头的三面墙，bool
    move_step(rel, heading)                # rel: 0=前 1=左 2=右 3=后
"""
import math
import random
import threading
import time

from maze_model import N, E, S, W, DIRS, OPPOSITE

FRONT, LEFT, RIGHT, BACK = 0, 1, 2, 3


def rel_to_abs(heading, rel):
    cw = [N, E, S, W]
    if rel == FRONT:
        return heading
    if rel == RIGHT:
        return cw[(cw.index(heading) + 1) % 4]
    if rel == BACK:
        return OPPOSITE[heading]
    return cw[(cw.index(heading) + 3) % 4]


class SimRobot:
    """理想（或带噪）传感器 + 完美运动的离线机器人。"""

    def __init__(self, truth, noise=0.0, seed=0, latency=0):
        self.truth = truth
        self.noise = noise            # 误判概率
        self.rng = random.Random(seed)
        self.latency = latency        # 墙壁信息滞后步数（0=实时）
        self._hist = []               # (pos, heading, walls)
        cell = None

    def set_pose(self, pos, heading):
        self.pos = pos
        self.heading = heading

    def read_walls(self):
        i, j = self.pos
        w = self.truth.walls[i][j]
        fl = bool(w & rel_to_abs(self.heading, LEFT))
        ff = bool(w & rel_to_abs(self.heading, FRONT))
        fr = bool(w & rel_to_abs(self.heading, RIGHT))
        if self.noise:
            fl = fl ^ (self.rng.random() < self.noise)
            ff = ff ^ (self.rng.random() < self.noise)
            fr = fr ^ (self.rng.random() < self.noise)
        return fl, ff, fr

    def move_step(self, rel, heading):
        d = rel_to_abs(heading, rel)
        di, dj = DIRS[d]
        self.pos = (self.pos[0] + di, self.pos[1] + dj)
        self.heading = d


class PoseNoiseRobot:
    """物理位置精确、里程计估计带漂移（模拟打滑/编码器误差）。

    算法用 est / est_heading 作为"自己以为在哪"，墙壁仍从物理位置真实读取，
    因此能评估定位漂移对建图与冲刺的影响。
    """

    def __init__(self, truth, cell_slip=0.0, noise=0.0, seed=0):
        self.phys = SimRobot(truth, noise=noise, seed=seed)
        self.rng = random.Random(seed + 7919)
        self.cell_slip = cell_slip
        self.est = (0, 0)
        self.est_heading = N

    def set_pose(self, pos, heading):
        self.phys.set_pose(pos, heading)
        self.est = pos
        self.est_heading = heading

    def read_walls(self):
        return self.phys.read_walls()

    def move_step(self, rel, heading):
        self.phys.move_step(rel, heading)
        d = rel_to_abs(heading, rel)
        di, dj = DIRS[d]
        ei, ej = self.est[0] + di, self.est[1] + dj
        if self.rng.random() < self.cell_slip:
            # 里程计漏计/多计一格（垂直于行进方向），模拟横向打滑
            if self.rng.random() < 0.5:
                ei, ej = ei + dj, ej - di
            else:
                ei, ej = ei - dj, ej + di
        self.est = (max(-2, min(17, ei)), max(-2, min(17, ej)))
        self.est_heading = d


class RosRobot:
    """对接仿真评测系统：/scan + /odom 输入，/cmd_vel 输出。

    坐标约定与 contest.py 一致：
        yaw = euler_z + pi  (范围 0..2pi)
        前进方向 4.71 / 右 3.0772 / 后 1.5072 / 左 0(或 6.28)
    """

    def __init__(self, config_ini=None, speed_x=0.1, speed_z=0.35, verbose=False,
                 detect_dist=0.15, decel=0.8):
        import rclpy
        from rclpy.node import Node
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import LaserScan

        self.rclpy = rclpy
        self.Node = Node
        self.Twist = Twist
        self.verbose = verbose

        cfg = self._load_config(config_ini)
        self.thr_l = cfg.get("threshold_l", 0.085)
        self.thr_f = cfg.get("threshold_f", 0.078)
        self.thr_r = cfg.get("threshold_r", 0.085)
        self.thr_side_far = cfg.get("threshold_lf", 0.118)
        # 建图用的"有墙"判定距离：单元格半宽 0.09，墙厚 0.006，
        # 侧墙实测 ≈0.084，无墙侧 ≥0.17，用 0.15 有足够裕度。
        self.detect_dist = detect_dist
        self.decel = decel        # 实测等效制动减速度 m/s^2（轮子加速度上限导致）
        self._closed = False
        self.blocksize = cfg.get("blocksize", 0.178)
        self.kp = cfg.get("kp", 10.0)
        self.speed_x = speed_x
        self.speed_z = speed_z
        self.fyaw = cfg.get("fyaw", 4.71)
        self.ryaw = cfg.get("ryaw", 3.0772)
        self.byaw = cfg.get("byaw", 1.5072)

        self.l_dis = self.fl_dis = self.f_dis = self.fr_dis = self.r_dis = 0.0
        self.x = self.y = 0.0
        self.yaw = 0.0
        self._odom_ready = False

        self.node = Node("baseline_floodfill")
        self.node.create_subscription(LaserScan, "/scan", self._laser, 10)
        self.node.create_subscription(Odometry, "/odom", self._odom, 10)
        self.pub = self.node.create_publisher(Twist, "/cmd_vel", 10)
        self.rate = self.node.create_rate(100.0)
        self._thread = None

    # -------- 配置 --------
    @staticmethod
    def _load_config(path):
        from configparser import ConfigParser
        import os
        out = {}
        if path and os.path.exists(path):
            c = ConfigParser()
            with open(path) as f:
                c.read_file(f)
            for k in ("threshold_l", "threshold_lf", "threshold_f", "threshold_rf",
                      "threshold_r", "blocksize", "kp", "fyaw", "ryaw", "byaw"):
                if c.has_option("threshold", k):
                    out[k] = float(c.get("threshold", k))
        return out

    # -------- 回调 --------
    # 单点取样（模板里的 340/270/180/90/20）在车体稍有偏转时极不稳定，
    # 这里改用固定角度区间的最小值，抗噪且与"墙是否在侧面"语义一致。
    LASER_BANDS = {"l": (330, 359), "fl": (250, 290), "f": (165, 195),
                   "fr": (70, 110), "r": (0, 30)}

    def _laser(self, msg):
        r = msg.ranges
        lo = msg.range_min if msg.range_min > 0 else 0.0

        def band(key):
            a, b = self.LASER_BANDS[key]
            vals = sorted(v for v in r[a:b + 1] if v == v and v > lo)  # 过滤 nan/0
            if not vals:
                return float("inf")
            # Gazebo 偶发单条射线野值（相邻射线仍是远处值），取第 3 小值规避
            return vals[min(2, len(vals) - 1)]

        self.l_dis = band("l")
        self.fl_dis = band("fl")
        self.f_dis = band("f")
        self.fr_dis = band("fr")
        self.r_dis = band("r")

    def _odom(self, msg):
        p = msg.pose.pose.position
        self.x, self.y = p.x, p.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = (math.atan2(siny, cosy) + math.pi) % (2 * math.pi)
        if not self._odom_ready and (self.x or self.y):
            self._odom_ready = True

    # -------- 生命周期 --------
    def start(self):
        self._thread = threading.Thread(target=self.rclpy.spin, args=(self.node,), daemon=True)
        self._thread.start()
        t0 = time.time()
        while time.time() - t0 < 15:
            if self._odom_ready and self.l_dis > 0:
                return True
            time.sleep(0.05)
        return self._odom_ready

    def stop(self):
        self.publish(0.0, 0.0)

    def close(self):
        """安全收尾，避免 spin 线程与解释器析构冲突。"""
        if self._closed:
            return
        self._closed = True
        try:
            self.publish(0.0, 0.0)
            time.sleep(0.3)
            self.node.destroy_node()
        except Exception:
            pass

    def publish(self, vx, wz):
        m = self.Twist()
        m.linear.x = float(vx)
        m.angular.z = float(wz)
        self.pub.publish(m)

    # -------- 位姿回算（相当于编码器 + 对墙校正后的格子定位）--------
    CENTER0 = -1.35
    CELL = 0.18

    def cell_from_odom(self):
        i = int(round((self.x - self.CENTER0) / self.CELL))
        j = int(round((self.y - self.CENTER0) / self.CELL))
        return i, j

    def heading_from_yaw(self):
        y = self.yaw
        if abs(y - 4.7124) < 0.8:
            return N
        if abs(y - 3.1416) < 0.8:
            return E
        if abs(y - 1.5708) < 0.8:
            return S
        return W

    # -------- 传感器 --------
    def read_walls(self):
        d = self.detect_dist
        return (self.l_dis < d, self.f_dis < d, self.r_dis < d)

    def turn_only(self, direction):
        """原地转到绝对朝向（供校验阶段逐格核查用）。"""
        self.turn_to_dir(direction)

    def centering_wz(self, target=0.09):
        """沿墙对中：两侧有墙用差值，只有一侧则贴墙保持 target 距离。"""
        near = 0.115
        lw, rw = self.l_dis < near, self.r_dis < near
        if lw and rw:
            e = self.l_dis - self.r_dis
        elif lw:
            e = self.l_dis - target
        elif rw:
            e = target - self.r_dis
        else:
            return 0.0
        wz = 0.5 * self.kp * e
        return max(-0.6, min(0.6, wz))

    def _posture(self):
        """沿墙 PI 修正，返回 angular.z。"""
        if self.f_dis > 0.09:
            if self.fl_dis < self.thr_side_far:
                return 0.5 * (self.fl_dis - self.thr_side_far) * self.kp
            if self.fr_dis < self.thr_side_far:
                return -0.5 * (self.fr_dis - self.thr_side_far) * self.kp
            return 0.0
        if self.l_dis < self.thr_l:
            return 0.5 * (self.l_dis - self.thr_l) * self.kp
        if self.r_dis < self.thr_r:
            return -0.5 * (self.r_dis - self.thr_r) * self.kp
        return 0.0

    # -------- 运动 --------
    def _hold(self, vx, wz, timeout=6.0, cond=None):
        t0 = time.time()
        while self.rclpy.ok():
            self.publish(vx, wz)
            self.rate.sleep()
            if cond is not None and cond():
                break
            if time.time() - t0 > timeout:
                break
        self.publish(0.0, 0.0)
        for _ in range(5):
            self.rate.sleep()

    def world_yaw(self):
        """偏航角转成常规数学角 (-pi, pi]，+x 为 0。"""
        y = self.yaw - math.pi
        while y <= -math.pi:
            y += 2 * math.pi
        while y > math.pi:
            y -= 2 * math.pi
        return y

    @staticmethod
    def _wrap(a):
        while a <= -math.pi:
            a += 2 * math.pi
        while a > math.pi:
            a -= 2 * math.pi
        return a

    def turn_to_dir(self, direction):
        """转到绝对朝向（北/东/南/西），闭环到 ±0.04 rad。"""
        target_world = {N: math.pi / 2, E: 0.0, S: -math.pi / 2, W: math.pi}[direction]
        t0 = time.time()
        while self.rclpy.ok():
            err = self._wrap(target_world - self.world_yaw())
            if abs(err) < 0.04:
                break
            if time.time() - t0 > 4.0:
                break
            self.publish(0.0, max(-self.speed_z, min(self.speed_z, 2.5 * err)))
            self.rate.sleep()
        self.publish(0.0, 0.0)
        for _ in range(6):
            self.rate.sleep()

    def goto_point(self, tx, ty, timeout=6.0, tol=0.006):
        """点到点闭环：同时修正朝向与横向偏差。"""
        t0 = time.time()
        while self.rclpy.ok():
            ex, ey = tx - self.x, ty - self.y
            dist = math.hypot(ex, ey)
            if dist < tol:
                break
            if time.time() - t0 > timeout:
                break
            err = self._wrap(math.atan2(ey, ex) - self.world_yaw())
            # 接近目标时减弱转向修正，避免方位角在短基线上病态放大
            wz = max(-1.0, min(1.0, 3.0 * err)) * min(1.0, dist / 0.06)
            if abs(err) > 0.9:                      # 方向差太多先原地转
                self.publish(0.0, wz)
            else:
                v = max(0.03, min(self.speed_x, 1.5 * dist))
                self.publish(v, wz)
            self.rate.sleep()
        self.publish(0.0, 0.0)
        for _ in range(8):
            self.rate.sleep()

    def goto_cell(self, i, j, **kw):
        self.goto_point(self.CENTER0 + self.CELL * i,
                        self.CENTER0 + self.CELL * j, **kw)

    def move_step(self, rel, heading):
        """按相对方向走一格：先转向，再闭环走到目标格中心。"""
        d = rel_to_abs(heading, rel)
        if rel != FRONT:
            self.turn_to_dir(d)
        i, j = self.cell_from_odom()
        di, dj = DIRS[d]
        self.goto_cell(i + di, j + dj)
        # 到点后把车头对齐到行驶方向，防止姿态误差逐格累积
        target_world = {N: math.pi / 2, E: 0.0, S: -math.pi / 2, W: math.pi}[d]
        if abs(self._wrap(target_world - self.world_yaw())) > 0.07:
            self.turn_to_dir(d)

