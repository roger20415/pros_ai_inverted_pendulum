from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from state_data_store import StateDataStore

class InvertedPendulumSubscriberNode(Node, StateDataStore):
    def __init__(self):
        super().__init__("inverted_pendulum_subscriber_node")
        self.get_logger().info("Start inverted pendulum subscriber node.")

        self._state_data_store = StateDataStore()

        self._calf_angle_subscriber = self.create_subscription(
            Float32MultiArray,
            "/inverted_pendulum_calf_angle",
            self._inverted_pendulum_calf_angle_subscribe_callback,
            1
        )

    def _inverted_pendulum_calf_angle_subscribe_callback(self, msg: Float32MultiArray) -> None:
        self._state_data_store.store_data("calf_angle", msg.data)