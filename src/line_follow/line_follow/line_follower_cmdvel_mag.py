import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt16MultiArray, Float32, String
from geometry_msgs.msg import Twist


# Small line follower that chooses front/back sensor based on direction.
# Publishes Twist messages to /cmd_vel_mag. Use /line_follow/direction (String)
# to change mode at runtime: 'forward' -> use sensor1, 'backward' -> use sensor2.

LINEAR_X = 0.08
K_ANG = 0.35
MAX_ANG = 0.2
THRESHOLD_SUM = 1.0
LOST_TIMEOUT = 0.3
CONTROL_RATE = 20.0

# If sensors are mounted reversed, toggle these
REVERSE_SENSOR_1 = False
REVERSE_SENSOR_2 = False


class LineFollowerCmdVelMag(Node):
    def __init__(self):
        super().__init__("line_follower_cmdvel_mag")

        self.declare_parameter("linear_x", LINEAR_X)
        self.declare_parameter("k_ang", K_ANG)
        self.declare_parameter("max_ang", MAX_ANG)
        self.declare_parameter("threshold_sum", THRESHOLD_SUM)
        self.declare_parameter("lost_timeout", LOST_TIMEOUT)
        self.declare_parameter("control_rate", CONTROL_RATE)
        self.declare_parameter("reverse_sensor_1", REVERSE_SENSOR_1)
        self.declare_parameter("reverse_sensor_2", REVERSE_SENSOR_2)
        self.declare_parameter("direction", "forward")

        self.linear_x = self.get_parameter("linear_x").get_parameter_value().double_value
        self.k_ang = self.get_parameter("k_ang").get_parameter_value().double_value
        self.max_ang = self.get_parameter("max_ang").get_parameter_value().double_value
        self.threshold_sum = self.get_parameter("threshold_sum").get_parameter_value().double_value
        self.lost_timeout = self.get_parameter("lost_timeout").get_parameter_value().double_value
        self.control_rate = self.get_parameter("control_rate").get_parameter_value().double_value
        self.reverse_sensor_1 = self.get_parameter("reverse_sensor_1").get_parameter_value().bool_value
        self.reverse_sensor_2 = self.get_parameter("reverse_sensor_2").get_parameter_value().bool_value
        self.direction = self.get_parameter("direction").get_parameter_value().string_value

        # subscriptions to both sensors
        self.sub1 = self.create_subscription(
            UInt16MultiArray, "/sensor1/analog16", self.cb_sensor1, 10
        )
        self.sub2 = self.create_subscription(
            UInt16MultiArray, "/sensor2/analog16", self.cb_sensor2, 10
        )

        # runtime direction control (string: 'forward' or 'backward')
        self.dir_sub = self.create_subscription(String, "/line_follow/direction", self.cb_direction, 10)

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel_mag", 10)
        self.err_pub = self.create_publisher(Float32, "/line_error", 10)

        self.sensor1 = None
        self.sensor2 = None
        self.last_t1 = 0.0
        self.last_t2 = 0.0

        # positions from left -> right (same as other node)
        self.positions = [-7.5, -6.5, -5.5, -4.5, -3.5, -2.5, -1.5, -0.5,
                           0.5,  1.5,  2.5,  3.5,  4.5,  5.5,  6.5,  7.5]

        self.timer = self.create_timer(1.0 / float(self.control_rate), self.control_loop)
        self.get_logger().info(f"line_follower_cmdvel_mag started, initial direction='{self.direction}'")

    def cb_sensor1(self, msg: UInt16MultiArray):
        self.sensor1 = list(msg.data)
        self.last_t1 = time.monotonic()

    def cb_sensor2(self, msg: UInt16MultiArray):
        self.sensor2 = list(msg.data)
        self.last_t2 = time.monotonic()

    def cb_direction(self, msg: String):
        val = msg.data.strip().lower()
        if val in ("forward", "backward"):
            self.direction = val
            self.get_logger().info(f"direction set to '{self.direction}' via topic")
        else:
            self.get_logger().warning(f"invalid direction '{msg.data}' received")

    def compute_error(self, data, reverse=False):
        if data is None or len(data) != 16:
            return None

        x = list(data)
        if reverse:
            x = x[::-1]

        s = sum(x)
        if s <= self.threshold_sum:
            return None

        weighted = 0.0
        for p, v in zip(self.positions, x):
            weighted += p * float(v)

        return weighted / float(s)

    def control_loop(self):
        now = time.monotonic()

        # choose which sensor to use based on self.direction
        use_sensor = 1 if self.direction == "forward" else 2

        error = None

        if use_sensor == 1 and (now - self.last_t1) < self.lost_timeout:
            error = self.compute_error(self.sensor1, self.reverse_sensor_1)
        elif use_sensor == 2 and (now - self.last_t2) < self.lost_timeout:
            error = self.compute_error(self.sensor2, self.reverse_sensor_2)

        cmd = Twist()

        if error is None:
            # stop if no line detected by the chosen sensor
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
        else:
            ang = -self.k_ang * error
            ang = max(-self.max_ang, min(self.max_ang, ang))

            # forward uses positive linear_x, backward uses negative
            lin = float(self.linear_x) if use_sensor == 1 else -float(self.linear_x)

            cmd.linear.x = lin
            cmd.angular.z = ang

            err_msg = Float32()
            err_msg.data = float(error)
            self.err_pub.publish(err_msg)

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = LineFollowerCmdVelMag()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
