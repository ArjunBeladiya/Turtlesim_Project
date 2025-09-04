import rclpy
from rclpy.node import Node
import random
import math
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class RandomMover(Node):
    def __init__(self):
        super().__init__('random_mover')

        # Parameters
        self.declare_parameter('turtle_name', 'turtle1')
        self.declare_parameter('other_turtle', 'turtle2')

        self.turtle_name = self.get_parameter('turtle_name').get_parameter_value().string_value
        self.other_turtle = self.get_parameter('other_turtle').get_parameter_value().string_value

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, f'/{self.turtle_name}/cmd_vel', 10)

        # Subscribers
        self.pose_sub = self.create_subscription(
            Pose, f'/{self.turtle_name}/pose', self.pose_callback, 10)
        self.other_pose_sub = self.create_subscription(
            Pose, f'/{self.other_turtle}/pose', self.other_pose_callback, 10)

        # Internal state
        self.pose = None
        self.other_pose = None
        self.timer = self.create_timer(0.1, self.move_random)

    def pose_callback(self, msg):
        self.pose = msg

    def other_pose_callback(self, msg):
        self.other_pose = msg

    def move_random(self):
        if self.pose is None:
            return
        msg = Twist()

        moved = False

        # --- Wall Avoidance ---
        if self.pose.x < 2.0 or self.pose.x > 9.0 or self.pose.y < 2.0 or self.pose.y > 9.0:
            msg.linear.x = 1.0
            msg.angular.z = 0.9
            moved = True

        # --- Turtle-Turtle Avoidance ---
        elif self.other_pose is not None:
            dist = math.sqrt(
                (self.pose.x - self.other_pose.x)**2 +
                (self.pose.y - self.other_pose.y)**2
            )
            if dist < 2.0:  # too close
                msg.linear.x = -1.0
                msg.angular.z = 0.9
                moved = True
                
        if not moved:
            msg.linear.x = random.uniform(0.5, 2.0)
            msg.angular.z = random.uniform(-0.9, 0.9)

        self.cmd_vel_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RandomMover()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
