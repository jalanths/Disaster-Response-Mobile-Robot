#!/usr/bin/env python3
"""
Disaster Rescue Rover — Waypoint Navigator
Navigates search/rescue waypoints one-by-one with:
  • Battery monitoring  — returns to base camp on LOW/CRITICAL
  • Hazard rerouting    — skips waypoint on CRITICAL detection nearby
  • Dashboard status    — publishes progress to /waypoint_status

Usage:
  python3 waypoint_navigator.py --go base_camp
  python3 waypoint_navigator.py --mission search_sweep
  python3 waypoint_navigator.py --mission full_coverage --loop
  python3 waypoint_navigator.py --list
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import yaml, math, argparse, os, sys, json, time

CONFIG = os.path.join(os.path.dirname(__file__), '../config/locations.yaml')

# Distance threshold (m) — skip waypoint if CRITICAL hazard this close
HAZARD_SKIP_DIST = 2.5
# Battery thresholds
BAT_LOW  = 25.0
BAT_CRIT = 10.0


def euler_to_quaternion(yaw):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def load_config(path):
    with open(os.path.abspath(path), 'r') as f:
        return yaml.safe_load(f)


def build_pose(node, loc):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = node.get_clock().now().to_msg()
    pose.pose.position.x = float(loc['x'])
    pose.pose.position.y = float(loc['y'])
    pose.pose.position.z = 0.0
    _, _, z, w = euler_to_quaternion(float(loc.get('yaw', 0.0)))
    pose.pose.orientation.z = z
    pose.pose.orientation.w = w
    return pose


class WaypointNavigator(Node):
    def __init__(self, waypoint_list, base_camp_loc, loop=False):
        super().__init__('waypoint_navigator')
        self.waypoint_list   = waypoint_list
        self.base_camp_loc   = base_camp_loc
        self.loop            = loop

        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Battery state
        self.battery_level  = None
        self.battery_status = 'OK'

        # Rescue state
        self.last_alert      = None
        self.skip_requested  = False
        self.survivors_found = 0
        self.zones_cleared   = 0

        # Subscribers
        self.create_subscription(String, '/battery_status',         self._bat_stat_cb,  10)
        self.create_subscription(String, '/battery_level_str',      self._bat_level_cb, 10)
        self.create_subscription(String, '/rescue_detection/alerts', self._alert_cb,     10)

        from std_msgs.msg import Float32
        self.create_subscription(Float32, '/battery_level', self._bat_float_cb, 10)

        # Publishers
        self.pub_status = self.create_publisher(String, '/waypoint_status', 10)
        self.pub_charge = self.create_publisher(String, '/battery_charge',  10)

    # ── Callbacks ─────────────────────────────────────────────────────────────
    def _bat_stat_cb(self, msg):
        self.battery_status = msg.data

    def _bat_level_cb(self, msg):
        try:
            self.battery_level = float(msg.data)
        except ValueError:
            pass

    def _bat_float_cb(self, msg):
        self.battery_level = round(float(msg.data), 1)

    def _alert_cb(self, msg):
        try:
            alert = json.loads(msg.data)
            self.last_alert = alert
            if alert.get('type') == 'SURVIVOR':
                self.survivors_found += 1
                self.get_logger().info(
                    f'Survivor #{self.survivors_found} detected at '
                    f'x={alert.get("robot_position", {}).get("x")}, '
                    f'y={alert.get("robot_position", {}).get("y")}')
            # Skip if CRITICAL hazard (fire/structural) too close
            if (alert.get('severity') == 'CRITICAL' and
                    alert.get('type') != 'SURVIVOR' and
                    alert.get('distance_m') is not None and
                    alert['distance_m'] <= HAZARD_SKIP_DIST):
                self.skip_requested = True
                self.get_logger().warn(
                    f'CRITICAL hazard {alert["distance_m"]}m away — '
                    f'flagging waypoint skip!')
        except Exception:
            pass

    # ── Dashboard publish ──────────────────────────────────────────────────────
    def _pub_status(self, name, idx, total):
        msg = String()
        msg.data = json.dumps({
            'name': name,
            'progress': f'{idx}/{total}',
            'survivors_found': self.survivors_found,
            'zones_cleared': self.zones_cleared,
        })
        self.pub_status.publish(msg)

    # ── Navigate to a single pose ──────────────────────────────────────────────
    def _go_to(self, name, loc, idx, total):
        """Send NavigateToPose goal, return True on success."""
        self.skip_requested = False
        self._pub_status(name, idx, total)
        self.get_logger().info(
            f'[{idx}/{total}] Navigating to {name} '
            f'(x={loc["x"]:.2f}, y={loc["y"]:.2f})  '
            f'Battery: {self.battery_level:.1f}%')

        goal = NavigateToPose.Goal()
        goal.pose = build_pose(self, loc)

        future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()

        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error(f'Goal rejected for {name}!')
            return False

        result_future = goal_handle.get_result_async()

        while not result_future.done():
            rclpy.spin_once(self, timeout_sec=0.2)

            if self.battery_status == 'CRITICAL':
                self.get_logger().error(
                    f'CRITICAL battery ({self.battery_level:.1f}%)! '
                    f'Cancelling and returning to base camp.')
                goal_handle.cancel_goal_async()
                return 'CRITICAL_BATTERY'

            if self.skip_requested:
                self.get_logger().warn(
                    f'Skipping {name} due to nearby CRITICAL hazard.')
                goal_handle.cancel_goal_async()
                return 'SKIP'

        result = result_future.result()
        if result and result.status == 4:   # SUCCEEDED
            self.zones_cleared += 1
            self.get_logger().info(f'Zone {name} cleared! (zones cleared: {self.zones_cleared})')
            return True
        else:
            self.get_logger().warn(f'Failed to reach {name} (status {result.status if result else "?"})')
            return False

    def _return_to_base(self):
        self.get_logger().info('Returning to base camp...')
        self._go_to('base_camp', self.base_camp_loc, 0, 0)
        charge_msg = String()
        charge_msg.data = 'start'
        self.pub_charge.publish(charge_msg)
        self.get_logger().info('At base camp — charging started.')

    # ── Main navigation loop ───────────────────────────────────────────────────
    def navigate(self):
        self.get_logger().info('Waiting for Nav2 NavigateToPose server...')
        self._client.wait_for_server()

        self.get_logger().info('Waiting for battery level...')
        while rclpy.ok() and self.battery_level is None:
            rclpy.spin_once(self, timeout_sec=0.5)
        self.get_logger().info(f'Battery confirmed: {self.battery_level:.1f}%')
        self.get_logger().info('Rescue rover ready. Starting mission.')

        total = len(self.waypoint_list)
        iteration = 0

        while rclpy.ok():
            iteration += 1
            if self.loop:
                self.get_logger().info(f'--- Sweep #{iteration} ---')

            for i, (name, loc) in enumerate(self.waypoint_list, start=1):
                if not rclpy.ok():
                    return

                if self.battery_status == 'CRITICAL':
                    self.get_logger().error(
                        f'CRITICAL battery before {name}! Returning to base camp.')
                    self._return_to_base()
                    return

                if self.battery_status == 'LOW':
                    self.get_logger().warn(
                        f'LOW battery ({self.battery_level:.1f}%) before {name}. '
                        f'Finishing current zone then returning to base camp.')
                    result = self._go_to(name, loc, i, total)
                    if result == 'CRITICAL_BATTERY':
                        self._return_to_base()
                        return
                    self._return_to_base()
                    return

                result = self._go_to(name, loc, i, total)

                if result == 'CRITICAL_BATTERY':
                    self._return_to_base()
                    return
                elif result == 'SKIP':
                    self.get_logger().info(f'Zone {name} skipped — moving on.')
                    continue

            self.get_logger().info(
                f'Mission complete — all zones visited. '
                f'Survivors found: {self.survivors_found}, '
                f'Zones cleared: {self.zones_cleared}')
            self._pub_status('base_camp', total, total)

            if not self.loop:
                break

        self.get_logger().info('Rescue mission finished.')


# ── CLI helpers ────────────────────────────────────────────────────────────────
def list_locations(config):
    print('\n--- Rescue Locations ---')
    for name, loc in config.get('locations', {}).items():
        print(f'  {name:<25} x={loc["x"]:.3f}, y={loc["y"]:.3f}')
    print('\n--- Missions ---')
    for name, stops in config.get('missions', {}).items():
        print(f'  {name:<25} -> {" -> ".join(stops)}')
    print()


def main():
    parser = argparse.ArgumentParser(description='Disaster Rescue Rover Navigator')
    parser.add_argument('--go',     type=str, help='Navigate to a named location')
    parser.add_argument('--mission',type=str, help='Run a predefined rescue mission')
    parser.add_argument('--loop',   action='store_true', help='Loop mission continuously')
    parser.add_argument('--list',   action='store_true', help='List all locations and missions')
    parser.add_argument('--config', default=CONFIG, help='Path to locations.yaml')
    args = parser.parse_args()

    config        = load_config(args.config)
    locations     = config.get('locations', {})
    missions      = config.get('missions', {})
    base_camp_loc = locations.get('base_camp', {'x': 0.0, 'y': 0.0, 'yaw': 0.0})

    if args.list:
        list_locations(config)
        sys.exit(0)

    waypoint_list = []

    if args.go:
        if args.go not in locations:
            print(f"Unknown location '{args.go}'. Use --list.")
            sys.exit(1)
        waypoint_list = [(args.go, locations[args.go])]

    elif args.mission:
        if args.mission not in missions:
            print(f"Unknown mission '{args.mission}'. Use --list.")
            sys.exit(1)
        for name in missions[args.mission]:
            if name not in locations:
                print(f"Location '{name}' not found.")
                sys.exit(1)
            waypoint_list.append((name, locations[name]))

    else:
        parser.print_help()
        sys.exit(1)

    rclpy.init()
    navigator = WaypointNavigator(waypoint_list, base_camp_loc, loop=args.loop)
    try:
        navigator.navigate()
    except KeyboardInterrupt:
        navigator.get_logger().info('Mission interrupted.')
    finally:
        navigator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
