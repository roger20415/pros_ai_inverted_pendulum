from rclpy.node import Node

from trajectory_msgs.msg import JointTrajectoryPoint

class ActionPublisherNode(Node):
    def __init__(self) -> None:
        super().__init__('inverted_pendulum_action_publisher_node')
        self.get_logger().info("Start inverted pendulum action publisher node.")
        
        self._target_joint_angles_publisher = self.create_publisher(
            JointTrajectoryPoint,
            '/inverted_pendulum_target_joint_angles',
            10
        )
          
    def publish_target_joint_angles(self, target_joint_angles: list[float]) -> None:
        print("publihs target_joint_angles: ", target_joint_angles)
        msg = JointTrajectoryPoint()
        msg.positions = target_joint_angles
        msg.velocities = [0.0] * len(target_joint_angles)
        self._target_joint_angles_publisher.publish(msg)
        