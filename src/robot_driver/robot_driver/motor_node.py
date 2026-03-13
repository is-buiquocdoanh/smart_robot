import math
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

from .usb_can_a import USBCanA
from .odom import Odometry as OdomCalc


LEFT_ID = 0x01
RIGHT_ID = 0x02

PORT = "/dev/ttyUSB0"

WHEEL_RADIUS = 0.074
WHEEL_BASE = 0.45
ENCODER_CPR = 10000

MAX_RPM = 3000


def linear_to_rpm(v):

    return v / WHEEL_RADIUS * 60 / (2*math.pi)


class MotorNode(Node):

    def __init__(self):

        super().__init__("motor_driver")

        self.cmd_sub = self.create_subscription(
            Twist,
            "/cmd_vel",
            self.cmd_callback,
            10
        )

        self.odom_pub = self.create_publisher(
            Odometry,
            "/odom",
            10
        )

        self.tf_broadcaster = TransformBroadcaster(self)

        self.dev = USBCanA(PORT)

        self.odom_calc = OdomCalc(
            WHEEL_RADIUS,
            WHEEL_BASE,
            ENCODER_CPR
        )

        self.vx = 0
        self.omega = 0

        self.last_cmd = time.time()

        self.timer = self.create_timer(
            0.02,
            self.update
        )

    def cmd_callback(self, msg):

        self.vx = msg.linear.x
        self.omega = msg.angular.z

        self.last_cmd = time.time()

    def twist_to_rpm(self):

        vL = self.vx - self.omega * WHEEL_BASE/2
        vR = self.vx + self.omega * WHEEL_BASE/2

        rpmL = int(linear_to_rpm(vL))
        rpmR = int(linear_to_rpm(vR))

        return rpmL, rpmR

    def send_speed(self, node_id, rpm):

        hi = (rpm >> 8) & 0xFF
        lo = rpm & 0xFF

        data = bytes([0x00,0x1A,0x10,hi,lo,0,0,1])

        self.dev.send_frame(node_id, data)

    def read_encoder(self, node_id):

        cmd = bytes.fromhex("00 2A E8 00 00 E9 00 00")

        self.dev.send_frame(node_id, cmd)

        frame = self.dev.read_frame_by_id(node_id)

        if frame is None:
            return None

        d = frame.data

        high16 = (d[3]<<8)|d[4]
        low16 = (d[6]<<8)|d[7]

        val = (high16<<16)|low16

        if val & 0x80000000:
            val -= 0x100000000

        return val

    def publish_odom(self, state):

        msg = Odometry()

        now = self.get_clock().now().to_msg()

        msg.header.stamp = now
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"

        msg.pose.pose.position.x = state.x
        msg.pose.pose.position.y = state.y

        msg.twist.twist.linear.x = state.vx
        msg.twist.twist.angular.z = state.omega

        self.odom_pub.publish(msg)

        t = TransformStamped()

        t.header.stamp = now
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"

        t.transform.translation.x = state.x
        t.transform.translation.y = state.y

        self.tf_broadcaster.sendTransform(t)

    def update(self):

        if time.time() - self.last_cmd > 0.5:

            self.vx = 0
            self.omega = 0

        rpmL, rpmR = self.twist_to_rpm()

        self.send_speed(LEFT_ID, rpmL)
        self.send_speed(RIGHT_ID, rpmR)

        encL = self.read_encoder(LEFT_ID)
        encR = self.read_encoder(RIGHT_ID)

        if encL is None:
            return

        dt = 0.02

        state = self.odom_calc.update(encL, encR, dt)

        self.publish_odom(state)


def main():

    rclpy.init()

    node = MotorNode()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()