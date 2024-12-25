from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from subscribe_data.unity_data_store import UnityDataStore

class DataSubscriberNode(Node):
    def __init__(self, unity_data_store: UnityDataStore) -> None:
        super().__init__("inverted_pendulum_data_subscriber_node")
        self.get_logger().info("Start inverted pendulum data subscriber node.")

        self._unity_data_store = unity_data_store

        self._state_subscriber = self.create_subscription(
            Float32MultiArray,
            "/state_topic",
            self._state_subscribe_callback,
            10
        )

    def _state_subscribe_callback(self, msg: Float32MultiArray) -> None:
        self._unity_data_store.split_and_store_received_array(msg)