#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult

from .motor_control import *
# odometry removed: not using wheel/odom due to inaccuracy
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

        # Runtime-configurable parameters
        port_param = self.declare_parameter("port", PORT).get_parameter_value().string_value
        baud_param = int(self.declare_parameter("baudrate", int(SERIAL_BAUD)).get_parameter_value().integer_value)
        gear_param = float(self.declare_parameter("gear_ratio", float(GEAR_RATIO)).get_parameter_value().double_value)
        invert_left = bool(self.declare_parameter("invert_left", False).get_parameter_value().bool_value)
        invert_right = bool(self.declare_parameter("invert_right", True).get_parameter_value().bool_value)

        # Apply initial runtime settings
        try:
            set_gear_ratio(gear_param)
        except Exception:
            self.get_logger().warning(f"Invalid gear_ratio parameter: {gear_param}")
        set_inversions(left=invert_left, right=invert_right)


        self.subscription = self.create_subscription(
            Twist,
            "/cmd_vel",
            self.cmd_callback,
            10
        )

        # odometry publisher and TF broadcaster removed (odom not used)

        self.dev = USBCanA(port=port_param, baudrate=baud_param)

        try:
            initialize_driver(self.dev, LEFT_ID)
            initialize_driver(self.dev, RIGHT_ID)
        except Exception as exc:
            self.get_logger().warning(f"Driver init warning: {exc}")

        # Odometry removed: not constructing DiffOdom because wheel/odom is inaccurate

        # Parameter change callback to allow updating gear_ratio and
        # inversion flags at runtime.
        def _on_set_params(params):
            successful = True
            reason = ''
            for p in params:
                try:
                    if p.name == 'gear_ratio':
                        # update gear ratio used for motor_control calculations
                        set_gear_ratio(p.value)
                    elif p.name == 'invert_left':
                        set_inversion(LEFT_ID, bool(p.value))
                    elif p.name == 'invert_right':
                        set_inversion(RIGHT_ID, bool(p.value))
                except Exception as exc:
                    successful = False
                    reason = str(exc)
            return SetParametersResult(successful=successful, reason=reason)

        self.add_on_set_parameters_callback(_on_set_params)

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

    # encoder/odometry removed: node will only send motor commands

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
            # Apply per-motor inversion (configurable)
            rpm_left_send = apply_inversion(LEFT_ID, rpm_left)
            rpm_right_send = apply_inversion(RIGHT_ID, rpm_right)

            run_speed_rpm(self.dev, LEFT_ID, rpm_left_send, wait_response=False)
            run_speed_rpm(self.dev, RIGHT_ID, rpm_right_send, wait_response=False)

        # Encoder reads and odometry publishing removed.
        # The node's responsibility is now limited to translating /cmd_vel -> motor commands.

    # (odometry publishing removed)


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
            