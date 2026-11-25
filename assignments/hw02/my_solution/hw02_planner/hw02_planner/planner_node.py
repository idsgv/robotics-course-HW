
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid

import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from scipy.ndimage import binary_dilation #for occupancy inflation of 0.3 meter radius

#Ido needs to go through this later on again
def inflate(occupied, resolution, radius=0.30):

    # Maximum number of neighboring cells to examine
    cells = int(np.ceil(radius / resolution)) + 1

    # Generate offsets around an occupied cell
    y, x = np.mgrid[-cells:cells+1, -cells:cells+1] 

    # Minimum distances between grid-cell areas
    dx = np.maximum(np.abs(x) - 1, 0) * resolution
    dy = np.maximum(np.abs(y) - 1, 0) * resolution

    # Which neighboring cells should become blocked?
    footprint = np.hypot(dx, dy) <= radius + 1e-9

    # Inflate the occupied cells
    return binary_dilation(occupied, structure=footprint)


class PlannerNode(Node):
    def __init__(self):
        super().__init__('hw02_planner')
        self.map = None
        self.configSpace = None
        self.start = None
        self.goal = None
        self.robot_pose = None
        self.ready = False
        self.declare_parameter('use_sim_time', True)
        self.map_received = False

        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )


        self.create_subscription(OccupancyGrid, '/map', self.on_map, qos)

        self.create_subscription(PoseStamped, '/hw02/start', self.on_start, qos)

        self.create_subscription(PoseStamped, '/hw02/goal', self.on_goal, qos)

        self.create_subscription(PoseStamped, '/hw02/robot_pose', self.on_robot_pose, 10)

        self.publisher = self.create_publisher(OccupancyGrid, '/hw02/cspace', qos)

        self.get_logger().info('Planner initialized')


    def on_map(self, msg):
        if self.map_received: #map is static
            return
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution

        #error handling:
        if width <= 0 or height <= 0 or not np.isfinite(resolution) or resolution <= 0:
            self.get_logger().error("Invalid map data received, map rejected.")
            return

        if len(msg.data) != width * height:
            self.get_logger().error("Map data size mismatch")
            return

        raw_map = np.array(msg.data).reshape((height, width))
        occupied = raw_map != 0

        print("Map received!")
        print("Shape:", raw_map.shape)
        print("Resolution:", resolution)
        print("Occupied cells:", np.sum(raw_map == 100))

        self.map = msg
        self.configSpace = np.where(inflate(occupied, resolution), 100, 0).astype(np.int8) #convert booleans into the actual 0-100 metric

        #publuish the configuration space
        cspace_msg = OccupancyGrid()
        cspace_msg.header = msg.header
        cspace_msg.info = msg.info
        cspace_msg.data = self.configSpace.flatten().tolist()
        self.publisher.publish(cspace_msg)
        self.map_received = True

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
