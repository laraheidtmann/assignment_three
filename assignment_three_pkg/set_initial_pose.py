import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from builtin_interfaces.msg import Time
import math

class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')

        self.pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )

        # publish once after 4 seconds (give AMCL time to start)
        self.timer = self.create_timer(4.0, self.publish_initial_pose)

    def publish_initial_pose(self):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.pose.pose.position.x = 0.0
        msg.pose.pose.position.y = 0.0
        msg.pose.pose.position.z = 0.0

        # quaternion yaw = 0
        msg.pose.pose.orientation.z = 0.0
        msg.pose.pose.orientation.w = 1.0

        # recommended covariance
        msg.pose.covariance = [
            0.25, 0.0,  0.0,  0.0,  0.0,  0.0,
            0.0,  0.25, 0.0,  0.0,  0.0,  0.0,
            0.0,  0.0,  9999.0, 0.0,  0.0,  0.0,
            0.0,  0.0,  0.0,  9999.0, 0.0,  0.0,
            0.0,  0.0,  0.0,  0.0,  9999.0, 0.0,
            0.0,  0.0,  0.0,  0.0,  0.0,  0.05
        ]

        self.pub.publish(msg)
        self.get_logger().info("Initial pose published automatically.")
        
        # Only publish once
        self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    node = InitialPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
