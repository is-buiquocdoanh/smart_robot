import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt16MultiArray, Float32
from geometry_msgs.msg import Twist


LINEAR_X = 0.15
K_ANG = 0.35
MAX_ANG = 1.2
THRESHOLD_SUM = 1.0
LOST_TIMEOUT = 0.3
CONTROL_RATE = 20.0

# Nếu lắp ngược đầu cảm biến thì đổi True
REVERSE_SENSOR_1 = False
REVERSE_SENSOR_2 = False


class LineFollowerCmdVel(Node):
    def __init__(self):
        super().__init__("line_follower_cmdvel")

        self.sub1 = self.create_subscription(
            UInt16MultiArray, "/sensor1/analog16", self.cb_sensor1, 10
        )
        self.sub2 = self.create_subscription(
            UInt16MultiArray, "/sensor2/analog16", self.cb_sensor2, 10
        )

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.err_pub = self.create_publisher(Float32, "/line_error", 10)

        self.sensor1 = None
        self.sensor2 = None
        self.last_t1 = 0.0
        self.last_t2 = 0.0

        # Vị trí kênh từ trái sang phải
        self.positions = [-7.5, -6.5, -5.5, -4.5, -3.5, -2.5, -1.5, -0.5,
                           0.5,  1.5,  2.5,  3.5,  4.5,  5.5,  6.5,  7.5]

        self.timer = self.create_timer(1.0 / CONTROL_RATE, self.control_loop)
        self.get_logger().info("line_follower_cmdvel started")

    def cb_sensor1(self, msg: UInt16MultiArray):
        self.sensor1 = list(msg.data)
        self.last_t1 = time.monotonic()

    def cb_sensor2(self, msg: UInt16MultiArray):
        self.sensor2 = list(msg.data)
        self.last_t2 = time.monotonic()

    def compute_error(self, data, reverse=False):
        if data is None or len(data) != 16:
            return None

        x = list(data)
        if reverse:
            x = x[::-1]

        s = sum(x)
        if s <= THRESHOLD_SUM:
            return None

        weighted = 0.0
        for p, v in zip(self.positions, x):
            weighted += p * float(v)

        return weighted / float(s)

    def control_loop(self):
        now = time.monotonic()

        e1 = None
        e2 = None

        if (now - self.last_t1) < LOST_TIMEOUT:
            e1 = self.compute_error(self.sensor1, REVERSE_SENSOR_1)

        if (now - self.last_t2) < LOST_TIMEOUT:
            e2 = self.compute_error(self.sensor2, REVERSE_SENSOR_2)

        errors = [e for e in [e1, e2] if e is not None]

        cmd = Twist()

        if len(errors) == 0:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
        else:
            error = sum(errors) / len(errors)

            ang = -K_ANG * error
            ang = max(-MAX_ANG, min(MAX_ANG, ang))

            cmd.linear.x = LINEAR_X
            cmd.angular.z = ang

            err_msg = Float32()
            err_msg.data = float(error)
            self.err_pub.publish(err_msg)

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = LineFollowerCmdVel()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
