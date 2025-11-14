import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose, Point
import math
from tf_transformations import euler_from_quaternion

class RobotObjectiveController(Node):
    def __init__(self):
        super().__init__('robot_controller')

        # Subscrições
        self.create_subscription(Pose, '/model/Robot/pose', self.pose_callback, 10)
        self.create_subscription(Point, '/RobotObjective', self.objective_callback, 10)

        # Publicador
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.pose = None
        self.objective = None
        self.timer = self.create_timer(0.1, self.control_loop)

    def pose_callback(self, msg):
        self.pose = msg

    def objective_callback(self, msg):
        self.objective = msg
        self.get_logger().info(f"Novo objetivo recebido: x={msg.x:.2f}, y={msg.y:.2f}")

    def control_loop(self):
        if not self.pose or not self.objective:
            return

        # --- Pose atual (no mundo) ---
        x = self.pose.position.x
        y = self.pose.position.y

        # Converte quaternion para yaw
        q = self.pose.orientation
        (_, _, yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])

        # --- Objetivo (no mundo) ---
        dx = self.objective.x - x
        dy = self.objective.y - y

        distance = math.sqrt(dx**2 + dy**2)
        angle_to_goal = math.atan2(dy, dx)

        # --- Erro angular em relação ao mundo ---
        angle_error = angle_to_goal - yaw

        # Normaliza o ângulo para [-pi, pi]
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

        # --- Controle proporcional simples ---
        linear_speed = 0.5 * distance
        angular_speed = 1.0 * angle_error

        cmd = Twist()
        cmd.linear.x = min(linear_speed, 0.5)
        cmd.angular.z = angular_speed
        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = RobotObjectiveController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
