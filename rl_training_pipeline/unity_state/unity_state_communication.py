from rclpy.node import Node
from std_msgs.msg import Bool

class UnityStateManagerNode(Node):
    def __init__(self) -> None:
        super().__init__('inverted_pendulum_unity_state_communicator_node')
        self.get_logger().info("Start inverted pendulum unity state communicator node.")
        
        self._is_training_paused: bool = False

        self._is_training_paused_subscriber = self.create_subscription(
            Bool,
            "/is_training_paused",
            self._subscribe_is_training_paused,
            1
        )

        self._reset_unity_scene_publisher = self.create_publisher(
            Bool,
            "/reset_unity",
            10
        )

        return None
    
    def publish_reset_unity_scene(self, should_reset_scene: bool) -> None:

        msg = Bool()
        msg.data = should_reset_scene
        self._reset_unity_scene_publisher.publish(msg)
        return None
        
    def get_is_training_paused(self) -> bool:

        return self._is_training_paused

    def set_is_training_paused(self, is_training_paused: bool) -> None:

        self._is_training_paused = is_training_paused
        return None
    
    def _subscribe_is_training_paused(self, msg: Bool) -> None:
        self._is_training_paused = msg.data
        return None