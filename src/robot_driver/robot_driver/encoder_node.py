#!/usr/bin/env python3

import time
from typing import Optional

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry as OdomMsg
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

from .motor_control import read_encoder, LEFT_ID, RIGHT_ID, WHEEL_RADIUS, WHEEL_BASE, ENCODER_CPR, PORT, SERIAL_BAUD, GEAR_RATIO
from .odom import Odometry as DiffOdom
from .usb_can_a import USBCanA
import rcl_interfaces.msg


class EncoderNode(Node):
    """ROS2 node that only reads encoders and publishes odometry.

    This node does NOT send any speed commands to the motor controller.
    It opens the serial CAN adapter and periodically requests the
    absolute encoder values from the left and right controllers, then
    updates a differential-drive odometry object and publishes the
    odometry message and TF.
    """

    def __init__(self):
        super().__init__("encoder_node")

        self.odom_pub = self.create_publisher(OdomMsg, "/wheel/odom", 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Open the serial/CAN adapter. Use the same defaults as motor_control
        # so this node can run on the same hardware without changes.
        port_param = self.declare_parameter("port", PORT).get_parameter_value().string_value
        baud_param = int(self.declare_parameter("baudrate", int(SERIAL_BAUD)).get_parameter_value().integer_value)
    gear_param = float(self.declare_parameter("gear_ratio", float(GEAR_RATIO)).get_parameter_value().double_value)

        self.get_logger().info(f"Encoder node using port={port_param} baud={baud_param} gear={gear_param}")

        self.dev = USBCanA(port=port_param, baudrate=baud_param)

        self.odom = DiffOdom(
            wheel_radius=WHEEL_RADIUS,
            wheel_base=WHEEL_BASE,
            encoder_cpr=ENCODER_CPR,
            gear_ratio=gear_param,
        )

        # allow updating gear_ratio at runtime
        def _on_set_params(params):
            for p in params:
                if p.name == 'gear_ratio':
                    try:
                        self.odom.GEAR = float(p.value)
                        return rcl_interfaces.msg.SetParametersResult(successful=True)
                    except Exception:
                        return rcl_interfaces.msg.SetParametersResult(successful=False, reason='invalid gear_ratio')
            return rcl_interfaces.msg.SetParametersResult(successful=True)

        self.add_on_set_parameters_callback(_on_set_params)

        # read rate: 20 Hz by default (same effective rate used in driver_node)
        self._rate_hz = 20.0
        self._period = 1.0 / self._rate_hz

        self.last_time = self.get_clock().now()

        self.timer = self.create_timer(self._period, self.update)

    def update(self) -> None:
        # Read encoder counts
        left = read_encoder(self.dev, LEFT_ID)
        right = read_encoder(self.dev, RIGHT_ID)

        if left is None or right is None:
            self.get_logger().warning(f"Encoder read failed: left={left} right={right}")
            return

        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        if dt <= 0:
            dt = self._period
        self.last_time = now

        state = self.odom.update(left, right, dt)

        # Publish nav_msgs/Odometry
        msg = OdomMsg()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"

        msg.pose.pose.position.x = state.x
        msg.pose.pose.position.y = state.y

        msg.twist.twist.linear.x = state.vx
        msg.twist.twist.angular.z = state.omega

        self.odom_pub.publish(msg)

        # Broadcast a TF transform for convenience
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = msg.header.frame_id
        t.child_frame_id = msg.child_frame_id
        t.transform.translation.x = state.x
        t.transform.translation.y = state.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(t)

    def destroy_node(self) -> None:  # override for clean close of serial
        try:
            self.dev.close()
        except Exception:
            pass
        super().destroy_node()


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = EncoderNode()

    try:
        rclpy.spin(node)
    finally:
        try:
            node.dev.close()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()