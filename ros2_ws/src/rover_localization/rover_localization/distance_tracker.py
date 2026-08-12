import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32
from std_srvs.srv import Trigger


class DistanceTrackerNode(Node):
    def __init__(self):
        super().__init__('distance_tracker_node')

        self.total_distance = 0.0
        self.last_position = None

        # Topic Publisher: Distance Covered (Rule 4)
        self.distance_pub = self.create_publisher(
            Float32, '/rover/distance_covered', 10
        )

        # Topic Subscriber: 6-DOF Corrected Pose from Pose Fusion (Person 3)
        self.subscription = self.create_subscription(
            PoseStamped, '/rover/corrected_pose', self.pose_callback, 10
        )

        # Service: Allow external nodes to reset distance counter to zero (Rule 4)
        self.reset_service = self.create_service(
            Trigger, '/rover/reset_distance', self.reset_distance_callback
        )

        self.get_logger().info('Distance Tracker Node initialized (Topics + Reset Service active).')

    def pose_callback(self, msg: PoseStamped):
        # Extract 6-DOF Position (x, y, z)
        current_pos = msg.pose.position

        if self.last_position is not None:
            # 3D Euclidean distance formula for 6-DOF position movement
            dx = current_pos.x - self.last_position.x
            dy = current_pos.y - self.last_position.y
            dz = current_pos.z - self.last_position.z
            delta = math.sqrt(dx * dx + dy * dy + dz * dz)

            # Filter out minor static sensor noise (below 1 mm)
            if delta > 0.001:
                self.total_distance += delta

        self.last_position = current_pos

        # Publish total distance covered
        dist_msg = Float32()
        dist_msg.data = float(self.total_distance)
        self.distance_pub.publish(dist_msg)

    def reset_distance_callback(self, request, response):
        """Service handler to reset total distance back to 0.0 meters."""
        self.total_distance = 0.0
        self.last_position = None
        response.success = True
        response.message = "Rover distance counter successfully reset to 0.0 meters."
        self.get_logger().info(response.message)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = DistanceTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()