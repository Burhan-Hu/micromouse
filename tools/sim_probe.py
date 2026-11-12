"""快速自检：打印五个激光索引、里程计、并抓一张俯视图。"""
import math, os, sys, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image
from nav_msgs.msg import Odometry
from PIL import Image as PImage

class Probe(Node):
    def __init__(s):
        super().__init__('probe')
        s.scan = s.odom = s.img = None
        s.create_subscription(LaserScan, '/scan', lambda m: setattr(s, 'scan', m), 10)
        s.create_subscription(Odometry, '/odom', lambda m: setattr(s, 'odom', m), 10)
        s.create_subscription(Image, '/camera_world/image_raw', lambda m: setattr(s, 'img', m), 5)

def main():
    out = sys.argv[1] if len(sys.argv) > 1 else '/tmp/sim_probe.png'
    rclpy.init()
    p = Probe()
    end = time.time() + 8
    while time.time() < end and (p.scan is None or p.odom is None):
        rclpy.spin_once(p, timeout_sec=0.1)
    if p.scan is None or p.odom is None:
        print("未收到 /scan 或 /odom —— 请先 tools/sim_up.sh"); return 1
    r = p.scan.ranges
    print("laser  l340=%.4f fl270=%.4f f180=%.4f fr90=%.4f r20=%.4f" % (r[340], r[270], r[180], r[90], r[20]))
    q = p.odom.pose.pose.orientation
    yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
    pos = p.odom.pose.pose.position
    print("odom   x=%.4f y=%.4f yaw+pi=%.4f" % (pos.x, pos.y, (yaw+math.pi) % (2*math.pi)))
    if p.img is not None:
        m = p.img
        PImage.frombytes('RGB', (m.width, m.height), bytes(m.data)).save(out)
        print("俯视图已保存:", out)
    return 0

if __name__ == '__main__':
    sys.exit(main())
