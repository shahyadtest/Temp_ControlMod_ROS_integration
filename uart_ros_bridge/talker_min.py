#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class TalkerMin(Node):
    def __init__(self):
        super().__init__('talker_min')
        self.pub = self.create_publisher(Float32, '/ref_cmd', 10)
        self.val = 50.0
        self.timer = self.create_timer(2.0, self.tick)
        self.get_logger().info("talker_min started: publishing /ref_cmd every 2s (demo).")

    def tick(self):
        msg = Float32()
        msg.data = float(self.val)
        self.pub.publish(msg)
        self.get_logger().info(f"Published /ref_cmd = {self.val:.2f}")
        # small toggle for demo
        self.val = 60.0 if self.val < 55.0 else 50.0

def main():
    rclpy.init()
    node = TalkerMin()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
