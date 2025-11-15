from controller import Supervisor
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class PosePublisher(Node):
    def __init__(self):
        super().__init__("pose_publisher")

        # ROS publishers for each robot
        self.leader_pub = self.create_publisher(PoseStamped, "/leader/true_pose", 10)
        self.follower_pub = self.create_publisher(PoseStamped, "/follower/true_pose", 10)


supervisor = Supervisor()
timestep = int(supervisor.getBasicTimeStep())

# Find robot nodes in the Webots scene tree
leader_node = supervisor.getFromDef("leader")
follower_node = supervisor.getFromDef("follower")

# Start ROS2
rclpy.init()
node = PosePublisher()

while supervisor.step(timestep) != -1:
    rclpy.spin_once(node, timeout_sec=0)

    # Leader pose
    if leader_node:
        pose = leader_node.getPosition()      # [x, y, z]
        node.leader_pub.publish(
            PoseStamped(
                header=node.get_clock().now().to_msg(),
                pose={
                    "position": {"x": pose[0], "y": pose[1], "z": pose[2]}
                }
            )
        )

    # Follower pose
    if follower_node:
        pose = follower_node.getPosition()
        node.follower_pub.publish(
            PoseStamped(
                header=node.get_clock().now().to_msg(),
                pose={
                    "position": {"x": pose[0], "y": pose[1], "z": pose[2]}
                }
            )
        )
