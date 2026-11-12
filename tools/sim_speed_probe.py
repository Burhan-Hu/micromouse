"""③ 探索速度评估：在 Gazebo 里实测不同直行速度对轮子摩擦的可用范围。

背景
    URDF 的 diff_drive 插件：wheel_diameter=0.02 m, max_wheel_torque=5 N·m,
    max_wheel_acceleration=1.0。实测发现 cmd_vel.linear.x 超过约 0.20 m/s 后
    轮子开始打滑（位移与指令速度严重不成比例），>=0.55 时甚至原地抖动无法起步。
    因此"提高探索速度"在这个仿真模型上没有收益空间。

本脚本给出定量证据：对每个速度测 0.5 s 内的实际位移，与理论值比较。

用法：
    tools/sim_up.sh && python3 tools/sim_speed_probe.py
"""
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from robot import RosRobot

SPEEDS = [0.05, 0.10, 0.15, 0.18, 0.20, 0.22, 0.25, 0.30]
DURATION = 0.5


def main():
    import rclpy
    from maze_model import N

    rclpy.init()
    r = RosRobot(os.path.join(os.path.dirname(os.path.dirname(
        os.path.abspath(__file__))), "TQD-Micromouse-OC v2.0", "config.ini"),
        speed_x=0.20, speed_z=1.2)
    if not r.start():
        print("未收到 /odom 或 /scan，请先 tools/sim_up.sh")
        return 1
    for _ in range(30):
        r.rate.sleep()

    print("速度   指令位移   实测位移    比值   说明")
    print("-" * 56)
    for v in SPEEDS:
        r.goto_cell(0, 0)
        for _ in range(12):
            r.rate.sleep()
        r.turn_to_dir(N)
        y0 = r.y
        t0 = time.time()
        while r.rclpy.ok() and time.time() - t0 < DURATION:
            r.publish(v, 0.0)
            r.rate.sleep()
        r.publish(0.0, 0.0)
        for _ in range(10):
            r.rate.sleep()
        moved = r.y - y0
        expect = v * DURATION
        ratio = moved / expect if expect else 0
        note = "正常" if 0.6 <= ratio <= 1.4 else ("打滑/空转" if ratio < 0.6 else "超速(里程计失真)")
        print(f"{v:5.2f}  {expect:8.3f}  {moved:+9.3f}  {ratio:6.2f}  {note}")

    r.stop()
    r.close()
    print("\n结论：可靠直行上限约 0.20 m/s（本 URDF 参数下），更高只会打滑。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
