#!/usr/bin/env python3
"""迷宫实时观察窗（固定全迷宫视野）

用一个独立窗口把仿真画面固定显示成"整个迷宫"，并高亮当前小车位置、
车头朝向与走过的轨迹，方便肉眼观察算法过程（不追踪、不缩放）。

前置
    需要仿真已在运行（gzserver 即可，有没有 3D 窗口都行）：
        tools/sim_gui.sh          # 带 Gazebo 3D 窗口
        tools/sim_up.sh           # 只要物理 + 传感器

用法
    python3 tools/maze_view.py                     # 默认 820x820，20 FPS
    python3 tools/maze_view.py --size 1100         # 更大的窗口
    python3 tools/maze_view.py --no-trail          # 不画轨迹
    python3 tools/maze_view.py --record run.mp4    # 同时录制视频
    python3 tools/maze_view.py --topic /camera_urdf/image_raw   # 换成别的相机

按键
    Q / Esc   退出
    SPACE     暂停 / 继续（暂停时仍继续接收数据）
    T         显示 / 隐藏 轨迹
    R         清空轨迹
    L         显示 / 隐藏 激光点

说明
    画面来自仿真自带的俯视相机 /camera_world/image_raw（约 1000x1000），
    其像素范围正好对应迷宫外框 ±1.446 m，无需额外标定。
"""
import argparse
import math
import os
import sys
import threading
import time
import tkinter as tk

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rclpy
from PIL import Image, ImageDraw, ImageTk
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image as RosImage
from sensor_msgs.msg import LaserScan

MAZE_SPAN = 2.892        # 迷宫外框边长（米），俯视相机正好覆盖 ±SPAN/2
CELL = 0.18
CENTER0 = -1.35          # 单元格(0,0)中心的世界坐标


class Source(Node):
    """收集显示所需的全部数据。"""

    def __init__(self, topic):
        super().__init__('maze_viewer')
        self.img = None
        self.pos = None
        self.yaw = 0.0
        self.scan = None
        self.n_img = 0
        self.n_odom = 0
        self.create_subscription(RosImage, topic, self._on_img, 5)
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self.create_subscription(LaserScan, '/scan', self._on_scan, 10)

    def _on_img(self, m):
        self.img = m
        self.n_img += 1

    def _on_odom(self, m):
        p = m.pose.pose.position
        self.pos = (p.x, p.y)
        q = m.pose.pose.orientation
        self.yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                              1 - 2 * (q.y * q.y + q.z * q.z))
        self.n_odom += 1

    def _on_scan(self, m):
        self.scan = m


class Viewer:
    def __init__(self, root, src, args):
        self.root, self.src, self.args = root, src, args
        self.paused = False
        self.show_trail = not args.no_trail
        self.show_laser = not args.no_laser
        self.trail = []
        self.photo = None
        self.closed = False
        self.writer = None

        root.title('电脑鼠 · 迷宫观察窗   [Q退出 SPACE暂停 T轨迹 L激光 R清空]')
        try:
            root.geometry('%+d%+d' % (args.x, args.y))   # 固定位置，别跑到别的屏
        except Exception:
            pass
        try:
            if not args.no_topmost:
                root.attributes('-topmost', True)  # 置顶，避免被别的窗口盖住
            root.lift()
            root.focus_force()
        except Exception:
            pass
        root.configure(bg='#0d0d0f')
        self.canvas = tk.Label(root, bg='#0d0d0f', bd=0)
        self.canvas.pack(padx=8, pady=(8, 4))
        self.status = tk.Label(root, text='等待仿真数据…', fg='#8ef0c0',
                               bg='#0d0d0f', font=('monospace', args.font))
        self.status.pack(pady=(0, 8))

        root.bind('<Escape>', lambda e: self.close())
        root.bind('q', lambda e: self.close())
        root.bind('<space>', self.toggle_pause)
        root.bind('t', self.toggle_trail)
        root.bind('l', self.toggle_laser)
        root.bind('r', self.clear_trail)

        if args.record:
            try:
                import cv2
                fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                self.writer = cv2.VideoWriter(args.record, fourcc,
                                              args.fps, (args.size, args.size))
                print('[录制] ->', args.record)
            except Exception as exc:
                print('[录制] 不可用:', exc)

        self.root.after(10, self.tick)

    # ---------- 交互 ----------
    def toggle_pause(self, _=None):
        self.paused = not self.paused

    def toggle_trail(self, _=None):
        self.show_trail = not self.show_trail

    def toggle_laser(self, _=None):
        self.show_laser = not self.show_laser

    def clear_trail(self, _=None):
        self.trail.clear()

    def close(self):
        self.closed = True
        if self.writer is not None:
            try:
                self.writer.release()
            except Exception:
                pass
        try:
            self.root.destroy()
        except Exception:
            pass

    # ---------- 主循环 ----------
    def tick(self):
        if self.closed:
            return
        if not self.paused:
            try:
                self.render()
            except Exception as exc:
                self.status.config(text='渲染异常: %s' % exc)
        self.root.after(max(1, int(1000 / self.args.fps)), self.tick)

    def render(self):
        s = self.src
        if s.img is None or s.pos is None:
            self.status.config(text='等待 %s 与 /odom …' % self.args.topic)
            return

        args = self.args
        m = s.img
        frame = Image.frombytes('RGB', (m.width, m.height), bytes(m.data))
        frame = frame.resize((args.size, args.size), Image.LANCZOS)
        d = ImageDraw.Draw(frame)
        N = args.size
        scale = N / MAZE_SPAN          # 像素 / 米

        def w2px(wx, wy):
            return ((wx + MAZE_SPAN / 2) * scale,
                    (MAZE_SPAN / 2 - wy) * scale)

        x, y = s.pos
        rx, ry = w2px(x, y)

        # 轨迹
        if self.show_trail:
            if not self.trail or (abs(self.trail[-1][0] - x) > 0.01 or
                                  abs(self.trail[-1][1] - y) > 0.01):
                self.trail.append((x, y))
                if len(self.trail) > 20000:
                    del self.trail[:5000]
            if len(self.trail) > 1:
                d.line([w2px(px, py) for px, py in self.trail],
                       fill=(70, 170, 255), width=2)

        # 激光点（画在轨迹之上、车之下）
        if self.show_laser and s.scan is not None:
            sc = s.scan
            n = len(sc.ranges)
            inc = (sc.angle_max - sc.angle_min) / (n - 1)
            for k in range(0, n, 2):
                r = sc.ranges[k]
                if r != r or r <= 0.0 or r > 3.4:
                    continue
                a = s.yaw + sc.angle_min + k * inc
                gx, gy = x + r * math.cos(a), y + r * math.sin(a)
                px, py = w2px(gx, gy)
                if 0 <= px < N and 0 <= py < N:
                    d.point((px, py), fill=(120, 210, 255))

        # 车体：真实 9cm×6cm
        L, Wd = 0.09 * scale, 0.06 * scale
        L, Wd = max(L, 10), max(Wd, 7)
        th = -s.yaw                       # 图像 y 轴向下
        corners = [(rx + dx * math.cos(th) - dy * math.sin(th),
                    ry + dx * math.sin(th) + dy * math.cos(th))
                   for dx, dy in ((L / 2, Wd / 2), (L / 2, -Wd / 2),
                                  (-L / 2, -Wd / 2), (-L / 2, Wd / 2))]
        d.polygon(corners, fill=(60, 220, 130), outline=(255, 255, 255))

        # 高亮环，保证全迷宫视野下一眼能找到车
        R = max(14.0, 0.085 * scale)
        d.ellipse([rx - R, ry - R, rx + R, ry + R],
                  outline=(255, 215, 0), width=3)
        # 车头箭头
        tip = (rx + (R + 12) * math.cos(th), ry + (R + 12) * math.sin(th))
        d.line([(rx, ry), tip], fill=(255, 90, 90), width=4)

        cell = (int(round((x - CENTER0) / CELL)),
                int(round((y - CENTER0) / CELL)))
        self.status.config(
            text='格子(%2d,%2d)  坐标(%+.3f,%+.3f)  朝向%6.1f°  '
                 '画面帧%5d  位姿帧%5d%s%s'
                 % (cell[0], cell[1], x, y, math.degrees(s.yaw),
                    s.n_img, s.n_odom,
                    '  [暂停]' if self.paused else '',
                    '' if self.show_trail else '  [无轨迹]'))

        self.photo = ImageTk.PhotoImage(frame)
        self.canvas.configure(image=self.photo)

        if self.writer is not None:
            try:
                import cv2
                import numpy as np
                self.writer.write(cv2.cvtColor(np.array(frame), cv2.COLOR_RGB2BGR))
            except Exception:
                self.writer = None


def main():
    ap = argparse.ArgumentParser(description='迷宫实时观察窗（固定全迷宫视野）')
    ap.add_argument('--topic', default='/camera_world/image_raw',
                    help='画面来源相机话题（默认仿真自带俯视相机）')
    ap.add_argument('--size', type=int, default=820, help='窗口画面边长像素')
    ap.add_argument('--fps', type=int, default=20, help='刷新率')
    ap.add_argument('--font', type=int, default=12, help='状态栏字号')
    ap.add_argument('--no-trail', action='store_true', help='不显示轨迹')
    ap.add_argument('--no-laser', action='store_true', help='不显示激光点')
    ap.add_argument('--record', default=None, help='同时录制为 mp4')
    ap.add_argument('--no-topmost', action='store_true', help='不置顶窗口')
    ap.add_argument('--x', type=int, default=60, help='窗口左上角 x 坐标')
    ap.add_argument('--y', type=int, default=60, help='窗口左上角 y 坐标')
    args = ap.parse_args()

    rclpy.init()
    src = Source(args.topic)
    threading.Thread(target=rclpy.spin, args=(src,), daemon=True).start()

    root = tk.Tk()
    Viewer(root, src, args)
    print('[观察窗] 已打开，窗口标题：电脑鼠 · 迷宫观察窗', flush=True)
    try:
        root.mainloop()
    finally:
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
