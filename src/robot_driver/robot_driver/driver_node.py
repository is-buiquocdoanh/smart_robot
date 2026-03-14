#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry as OdomMsg
from geometry_msgs.msg import TransformStamped

from tf2_ros import TransformBroadcaster

from .motor_control import *
from .odom import Odometry as DiffOdom
from .usb_can_a import USBCanA
import time
import signal


"""
ROS2 node that bridges /cmd_vel -> motor controller.

Notes and debugging tips:
- The motor IO (serial/CAN) can block if you perform round-trip `transact`
    calls on every control loop tick. To keep the ROS timer responsive the
    node sends speed frames non-blocking (uses run_speed_rpm(..., wait_response=False)).
- Encoder reads and odometry updates are intentionally performed at a
    lower rate (controlled by `_ticks_per_encoder`) to reduce blocking.
- Tuning:
    - increase `_ticks_per_encoder` to lower CPU/IO usage further (but odom
        updates will be less frequent)
    - decrease `_ticks_per_encoder` to increase odom t Hz (but more serial reads)
    - If you need both high-rate odom and low-latency commands, consider
        moving serial I/O to a dedicated thread that reads/writes continuously.
"""


class TsdaDriver(Node):

    def __init__(self):

        super().__init__("tsda_driver")

        self.subscription = self.create_subscription(
            Twist,
            "/cmd_vel",
            self.cmd_callback,
            10
        )

        self.odom_pub = self.create_publisher(
            OdomMsg,
            "/wheel/odom",
            10
        )

        self.tf_broadcaster = TransformBroadcaster(self)

        self.dev = USBCanA(port=PORT, baudrate=SERIAL_BAUD)

        initialize_driver(self.dev, LEFT_ID)
        initialize_driver(self.dev, RIGHT_ID)

        self.odom = DiffOdom(
            wheel_radius=WHEEL_RADIUS,
            wheel_base=WHEEL_BASE,
            encoder_cpr=ENCODER_CPR
        )

        self.vx = 0.0
        self.omega = 0.0
        # Whether we should actively send speed commands to the motor
        # controllers. When False we still read encoders and publish odom
        # but do not send run_speed_rpm frames so motors are not driven.
        self.control_active = False
        # Small deadzone to treat very small velocities as zero
        self._vel_deadzone = 1e-4

        # run at 100 Hz; but avoid blocking serial calls every tick.
        # we'll send speed commands every tick (non-blocking) but only
        # read encoders and update odom at a lower rate.
        self.timer = self.create_timer(0.01, self.update) # 100 Hz

        # counter to reduce blocking encoder reads; read encoders every N ticks
        self._ticks = 0
        self._ticks_per_encoder = 5  # 100Hz / 5 -> 20 Hz encoder/odom update

        self.last_time = self.get_clock().now()

        # flag set by external signal handlers to request clean shutdown
        self._shutdown_requested = False

    def request_shutdown(self):
        # mark shutdown requested so external code can check
        self._shutdown_requested = True

    def cmd_callback(self, msg):

        vx = msg.linear.x
        omega = msg.angular.z

        # Update stored commands
        self.vx = vx
        self.omega = omega

        # Determine if this is a non-zero command (outside deadzone)
        nonzero = (abs(vx) > self._vel_deadzone) or (abs(omega) > self._vel_deadzone)

        if nonzero and not self.control_active:
            # Transition inactive -> active: start sending commands
            self.get_logger().debug("Control active: enabling motor commands")
            self.control_active = True

        if not nonzero and self.control_active:
            # Transition active -> inactive: send a stop sequence once and
            # then cease sending further speed frames so motors are free.
            self.get_logger().info("Control inactive: sending stop sequence and disabling active commands")
            try:
                # send a few zero RPMs non-blocking to increase chance of delivery
                for _ in range(3):
                    run_speed_rpm(self.dev, LEFT_ID, 0, wait_response=False)
                    run_speed_rpm(self.dev, RIGHT_ID, 0, wait_response=False)
                    time.sleep(0.02)
                # blocking stop command
                stop_motor(self.dev, LEFT_ID)
                stop_motor(self.dev, RIGHT_ID)
            except Exception as exc:  # pragma: no cover - runtime safety
                self.get_logger().warning(f"Failed to send stop sequence: {exc}")

            self.control_active = False

    def update(self):

        rpm_left, rpm_right = twist_to_rpm(self.vx, self.omega)

        # Send speed commands only when control_active is True. If control
        # is inactive we do not send frames so the motor controller is not
        # continuously commanded (allows free/coast behaviour).
        if self.control_active:
            # Send speed commands without waiting for a response to avoid
            # blocking the timer loop. The motor controller accepts frames
            # and will act on them even if we don't wait for an ACK.
            run_speed_rpm(self.dev, LEFT_ID, rpm_left, wait_response=False)
            run_speed_rpm(self.dev, RIGHT_ID, rpm_right, wait_response=False)

        # Only read encoders and update odometry at a reduced rate to
        # avoid blocking the 100 Hz control loop with serial reads.
        self._ticks += 1
        if self._ticks % self._ticks_per_encoder != 0:
            return

        left = read_encoder(self.dev, LEFT_ID)
        right = read_encoder(self.dev, RIGHT_ID)

        if left is None or right is None:
            # Log a warning so we can see in runtime why odom is not published.
            self.get_logger().warning(
                f"Encoder read failed: left={left} right={right} tick={self._ticks}"
            )
            return

        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        state = self.odom.update(left, right, dt)

        self.publish_odom(state)
        # Log at debug level that we published odometry (helps verify publisher)
        self.get_logger().debug(
            f"Published odom: x={state.x:.3f} y={state.y:.3f} vx={state.vx:.3f} omega={state.omega:.3f}"
        )

    def publish_odom(self, state):

        msg = OdomMsg()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"

        msg.pose.pose.position.x = state.x
        msg.pose.pose.position.y = state.y

        msg.twist.twist.linear.x = state.vx
        msg.twist.twist.angular.z = state.omega

        self.odom_pub.publish(msg)

        t = TransformStamped()

        t.header.stamp = msg.header.stamp
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"

        t.transform.translation.x = state.x
        t.transform.translation.y = state.y

        self.tf_broadcaster.sendTransform(t)


def main(args=None):

    rclpy.init(args=args)

    node = TsdaDriver()

    try:
        rclpy.spin(node)
    finally:
        # Ensure we send a safe shutdown sequence to the motor controllers
        # and close the serial device on shutdown so the motors are not
        # left in an enabled/holding state when the node exits.
        def _safe_shutdown(n: TsdaDriver):
            # 1) Send a few zero-RPM commands non-blocking to increase the
            # chance the controller receives a velocity=0 as the last
            # command.
            try:
                for _ in range(4):
                    run_speed_rpm(n.dev, LEFT_ID, 0, wait_response=False)
                    run_speed_rpm(n.dev, RIGHT_ID, 0, wait_response=False)
                    time.sleep(0.02)
            except Exception as exc:  # pragma: no cover - runtime safety
                n.get_logger().warning(f"Failed sending zero RPMs during shutdown: {exc}")

            # 2) Try to send a blocking STOP MOTOR frame (best-effort).
            try:
                stop_motor(n.dev, LEFT_ID)
                stop_motor(n.dev, RIGHT_ID)
            except Exception as exc:  # pragma: no cover - runtime safety
                n.get_logger().warning(f"Failed to send stop command on shutdown: {exc}")

            # 3) Close the serial device.
            try:
                n.dev.close()
            except Exception:
                pass

        try:
            _safe_shutdown(node)
        finally:
            # Destroy the node and shutdown rclpy
            try:
                node.destroy_node()
            except Exception:
                pass
            rclpy.shutdown()
            