#!/usr/bin/env python3
"""
Battery Simulator
Drains battery over time. Publishes level + status.
Waypoint navigator listens and returns to dock when low.

Topics:
  /battery_level  (Float32)  — 0.0 to 100.0
  /battery_status (String)   — OK / LOW / CRITICAL
  /battery_charge (String)   — publish "start" to begin charging
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String

DRAIN_RATE      = 0.15   # % per second  (~11 min full patrol)
CHARGE_RATE     = 0.5    # % per second when docked
LOW_THRESHOLD   = 25.0
CRIT_THRESHOLD  = 10.0


class BatterySimulator(Node):
    def __init__(self):
        super().__init__('battery_simulator')
        self.level    = 100.0
        self.charging = False

        self.pub_level  = self.create_publisher(Float32, '/battery_level',  10)
        self.pub_status = self.create_publisher(String,  '/battery_status', 10)

        self.create_subscription(String, '/battery_charge', self.charge_cb, 10)
        self.create_timer(1.0, self.tick)

        self.get_logger().info('Battery simulator started. Level: 100%')

    def charge_cb(self, msg):
        self.charging = (msg.data.strip().lower() == 'start')
        state = 'charging' if self.charging else 'discharging'
        self.get_logger().info(f'Battery {state}')

    def tick(self):
        if self.charging:
            self.level = min(100.0, self.level + CHARGE_RATE)
            if self.level >= 100.0:
                self.charging = False
                self.get_logger().info('Battery fully charged.')
        else:
            self.level = max(0.0, self.level - DRAIN_RATE)

        level_msg = Float32()
        level_msg.data = round(self.level, 2)
        self.pub_level.publish(level_msg)

        if self.level <= CRIT_THRESHOLD:
            status = 'CRITICAL'
        elif self.level <= LOW_THRESHOLD:
            status = 'LOW'
        else:
            status = 'OK'

        status_msg = String()
        status_msg.data = status
        self.pub_status.publish(status_msg)

        # Log at each threshold crossing
        pct = int(self.level)
        if pct in (75, 50, 25, 20, 15, 10, 5):
            self.get_logger().info(f'Battery: {self.level:.1f}% [{status}]')


def main():
    rclpy.init()
    node = BatterySimulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
