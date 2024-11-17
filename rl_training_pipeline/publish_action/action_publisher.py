from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectoryPoint

class ActionPublisherNode(Node):
    def __init__(self) -> None:
        super().__init__('inverted_pendulum_action_publisher_node')
        self.get_logger().info("Start inverted pendulum action publisher node.")
        
        self._joint_target_publisher = self.create_publisher(
            JointTrajectoryPoint,
            '/inverted_pendulum_joint_target',
            10
        )
        
        return None
        
    def publish_joint_targets(self, joint_targets: list[float]) -> None:
        msg = JointTrajectoryPoint()
        msg.positions = joint_targets
        msg.velocities = [0.0] * len(joint_targets)
        self._joint_target_publisher.publish(msg)
 
        return None