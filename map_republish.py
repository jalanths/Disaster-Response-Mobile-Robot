#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import OccupancyGrid

class MapRepublisher(Node):
    def __init__(self):
        super().__init__('map_republisher')

        transient_local_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )

        volatile_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )

        self.pub = self.create_publisher(OccupancyGrid, '/map_volatile', volatile_qos)
        self.sub = self.create_subscription(OccupancyGrid, '/map', self.callback, transient_local_qos)
        self.get_logger().info('Map republisher started')

    def callback(self, msg):
        self.pub.publish(msg)
        self.get_logger().info(f'Republished map: {msg.info.width}x{msg.info.height}')

def main():
    rclpy.init()
    node = MapRepublisher()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
