#!/usr/bin/env python3
"""Docking manager node:
 - subscribes to /battery_state and triggers auto-docking when below threshold
 - sends a Nav2 NavigateToPose goal to move to a predefined charging pose
 - after navigation succeeds, runs a simple ultrasonic-based reverse docking controller

This is a lightweight, opinionated implementation intended as a starting point.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose
import time


class DockingManager(Node):
    def __init__(self):
        super().__init__('docking_manager')

        # parameters
        self.declare_parameter('battery_topic', '/battery_state')
        self.declare_parameter('battery_threshold', 0.20)
        self.declare_parameter('charging_pose_frame', 'map')
        self.declare_parameter('charging_pose_x', 0.0)
        self.declare_parameter('charging_pose_y', 0.0)
        self.declare_parameter('charging_pose_yaw', 0.0)
        # docking method (default to line_follow)
        self.declare_parameter('docking_method', 'line_follow')
        # topic where line follower publishes its Twist (default cmd_vel_mag)
        self.declare_parameter('line_cmd_topic', '/cmd_vel_mag')

        battery_topic = self.get_parameter('battery_topic').value
        self.threshold = float(self.get_parameter('battery_threshold').value)

        self.charging_pose = PoseStamped()
        frame = self.get_parameter('charging_pose_frame').value
        x = float(self.get_parameter('charging_pose_x').value)
        y = float(self.get_parameter('charging_pose_y').value)
        yaw = float(self.get_parameter('charging_pose_yaw').value)

        # compute quaternion from yaw only (2D pose)
        import math
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        q = (0.0, 0.0, qz, qw)

        self.charging_pose.header.frame_id = frame
        self.charging_pose.pose.position.x = x
        self.charging_pose.pose.position.y = y
        self.charging_pose.pose.position.z = 0.0
        self.charging_pose.pose.orientation.x = q[0]
        self.charging_pose.pose.orientation.y = q[1]
        self.charging_pose.pose.orientation.z = q[2]
        self.charging_pose.pose.orientation.w = q[3]

        self.battery_sub = self.create_subscription(BatteryState, battery_topic, self.battery_cb, 10)

        # action client to Nav2
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # publisher to control line_follow direction topic
        self.line_dir_pub = self.create_publisher(String, '/line_follow/direction', 10)

        # monitor line follower cmd topic to detect stop (when it reaches end of tape)
        self.line_cmd_topic = self.get_parameter('line_cmd_topic').value
        self._last_line_cmd_time = None
        self._last_line_linear = None
        self._line_stop_detected = False
        self._line_monitor_timer = None
        # only subscribe when needed (to avoid unnecessary topics)
        self._line_cmd_sub = None

        # docking method
        self.docking_method = self.get_parameter('docking_method').value

        self.get_logger().info('DockingManager ready (line-follow docking)')

    def battery_cb(self, msg: BatteryState):
        try:
            pct = float(msg.percentage)
        except Exception:
            return
        self.get_logger().debug(f'Battery percentage: {pct:.2f}')
        if pct <= self.threshold:
            self.get_logger().info(f'Battery low ({pct:.2f}) <= threshold {self.threshold}, starting docking sequence')
            # start docking sequence
            self.start_docking_sequence()

    def start_docking_sequence(self):
        # 1) send nav2 goal to charging_pose
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('NavigateToPose action server not available')
            return

        goal_msg = NavigateToPose.Goal()
        # copy charging_pose
        goal_msg.pose = self.charging_pose

        send_goal_future = self._action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        try:
            goal_handle = future.result()
        except Exception as e:
            self.get_logger().error(f'Failed to send goal: {e}')
            return
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by nav2')
            return
        self.get_logger().info('Nav2 accepted goal, waiting for result...')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self._on_nav_done)

    def _on_nav_done(self, future):
        try:
            result = future.result().result
            status = future.result().status
        except Exception as e:
            self.get_logger().error(f'Nav2 call failed: {e}')
            return
        if status == 4:  # SUCCEEDED (action_msgs/GoalStatus.STATUS_SUCCEEDED == 4)
            self.get_logger().info('Navigation succeeded — starting docking method: %s' % self.docking_method)
            # small delay to stabilize
            time.sleep(0.5)
            # We only support line-follow docking here; if parameter differs, default to line_follow
            if str(self.docking_method).lower() in ('line_follow', 'line-follow', 'linefollow'):
                # publish direction 'backward' to start the line follower in reverse mode
                self._start_line_follow()
            else:
                self.get_logger().warn(f"Unknown docking_method '{self.docking_method}', defaulting to line_follow")
                self._start_line_follow()
        else:
            self.get_logger().warn(f'Navigation ended with status {status}, not starting docking')

    # ---- line-follow integration ----
    def _start_line_follow(self):
        # ensure we have a subscriber to the line follower cmd topic
        if self._line_cmd_sub is None:
            self._line_cmd_sub = self.create_subscription(Twist, self.line_cmd_topic, self._line_cmd_cb, 10)

        # reset monitoring state
        self._last_line_cmd_time = None
        self._last_line_linear = None
        self._line_stop_detected = False

        # publish backward direction to start following backwards
        msg = String()
        msg.data = 'backward'
        self.line_dir_pub.publish(msg)
        self.get_logger().info('Published /line_follow/direction=backward')

        # start a short timer to monitor when the line follower stops moving (indicates end of tape)
        if self._line_monitor_timer is None:
            self._line_monitor_timer = self.create_timer(0.1, self._check_line_stop)

    def _line_cmd_cb(self, msg: Twist):
        # store last linear.x and time
        self._last_line_linear = float(msg.linear.x)
        self._last_line_cmd_time = self.get_clock().now()

    def _check_line_stop(self):
        # if we haven't received any cmd recently, ignore
        if self._last_line_cmd_time is None:
            return
        now = self.get_clock().now()
        age = (now - self._last_line_cmd_time).nanoseconds / 1e9
        # if older than 1s, consider it lost and ignore
        if age > 1.0:
            return

        # if linear velocity is near zero for a short period, mark as docked
        if abs(self._last_line_linear) < 0.01:
            # we consider docked when commanded speed goes to ~0
            if not self._line_stop_detected:
                self._line_stop_detected = True
                self.get_logger().info('Line follower stopped (possible docking complete)')
                # publish a docking state message
                try:
                    ds = String()
                    ds.data = 'docked'
                    # lazily create docking_state publisher
                    if not hasattr(self, 'docking_state_pub'):
                        self.docking_state_pub = self.create_publisher(String, '/docking_state', 10)
                    self.docking_state_pub.publish(ds)
                except Exception:
                    pass
                # stop monitoring timer after detection
                if self._line_monitor_timer is not None:
                    self._line_monitor_timer.cancel()
                    self._line_monitor_timer = None
        else:
            # reset detection if still moving
            self._line_stop_detected = False


def main(args=None):
    rclpy.init(args=args)
    node = DockingManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # cleanup
        node.get_logger().info('Shutting down DockingManager')
        # no executor to shutdown (line-follow only)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
