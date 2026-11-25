
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from hw02_sim.map_server import MAP_QOS


class PlannerNode(Node):
    def __init__(self):
        super().__init__('hw02_planner')
        self.map = None
        self.start = None
        self.goal = None
        self.robot_pose = None
        self.ready = False

        self.create_subscription(OccupancyGrid, '/map', self.on_map, MAP_QOS)

        self.create_subscription(PoseStamped, '/hw02/start', self.on_start, MAP_QOS)

        self.create_subscription(PoseStamped, '/hw02/goal', self.on_goal, MAP_QOS)

        self.create_subscription(PoseStamped, '/hw02/robot_pose', self.on_robot_pose, 10)

        self.get_logger().info('Planner initialized')

    def on_map(self, msg):
        if self.map is None:
            self.get_logger().info('PASS: map received')
        self.map = msg
        self.check_readiness()

    def on_start(self, msg):
        if self.start is None:
            self.get_logger().info('PASS: start received')
        self.start = msg
        self.check_readiness()

    def on_goal(self, msg):
        if self.goal is None:
            self.get_logger().info('PASS: goal received')
        self.goal = msg
        self.check_readiness()

    def on_robot_pose(self, msg):
        if self.robot_pose is None:
            self.get_logger().info('PASS: robot pose received')
        self.robot_pose = msg
        self.check_readiness()

    def check_readiness(self):
        if not self.ready and all(
            msg is not None for msg in (
                self.map,
                self.start,
                self.goal,
                self.robot_pose
            )
        ):
            self.ready = True
            self.get_logger().info('PASS: planner ready')


def main(args=None):
    rclpy.init(args=args)
    node = PlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
