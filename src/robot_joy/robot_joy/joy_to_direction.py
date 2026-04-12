import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import String


class JoyToDirection(Node):
    """Subscribe to /joy and publish /line_follow/direction as String.

    Parameters (ROS parameters):
      forward_button (int): button index to publish 'forward' (default 4)
      backward_button (int): button index to publish 'backward' (default 5)
      topic (str): output topic (default '/line_follow/direction')
    """

    def __init__(self):
        super().__init__("joy_to_direction")

        self.declare_parameter("forward_button", 0)
        self.declare_parameter("backward_button", 2)
        self.declare_parameter("topic", "/line_follow/direction")

        self.forward_button = self.get_parameter("forward_button").get_parameter_value().integer_value
        self.backward_button = self.get_parameter("backward_button").get_parameter_value().integer_value
        self.out_topic = self.get_parameter("topic").get_parameter_value().string_value

        self.pub = self.create_publisher(String, self.out_topic, 10)
        self.sub = self.create_subscription(Joy, "/joy", self.cb_joy, 10)

        # keep previous button states to detect rising edge
        self.prev_buttons = []

        self.get_logger().info(f"joy_to_direction started: forward_button={self.forward_button}, backward_button={self.backward_button}")

    def cb_joy(self, msg: Joy):
        # ensure prev_buttons has same length
        if len(self.prev_buttons) != len(msg.buttons):
            self.prev_buttons = [0] * len(msg.buttons)

        # handle forward button
        for name, idx in (("forward", self.forward_button), ("backward", self.backward_button)):
            if 0 <= idx < len(msg.buttons):
                cur = msg.buttons[idx]
                prev = self.prev_buttons[idx]
                # rising edge
                if cur and not prev:
                    s = String()
                    s.data = "forward" if name == "forward" else "backward"
                    self.pub.publish(s)
                    self.get_logger().info(f"published direction '{s.data}' from button {idx}")

        # update prev_buttons
        self.prev_buttons = list(msg.buttons)


def main(args=None):
    rclpy.init(args=args)
    node = JoyToDirection()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
