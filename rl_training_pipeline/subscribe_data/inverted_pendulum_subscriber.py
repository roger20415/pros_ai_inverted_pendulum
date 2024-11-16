from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from subscribe_data.unity_data_store import UnityDataStore

class InvertedPendulumSubscriberNode(Node):
    def __init__(self, unity_data_store: UnityDataStore):
        super().__init__("inverted_pendulum_subscriber_node")
        self.get_logger().info("Start inverted pendulum subscriber node.")

        self._unity_data_store = unity_data_store

        self._calf_angle_subscriber = self.create_subscription(
            Float32MultiArray,
            "/inverted_pendulum_calf_angle",
            self._inverted_pendulum_calf_angle_subscribe_callback,
            1
        )

    def _inverted_pendulum_calf_angle_subscribe_callback(self, msg: Float32MultiArray) -> None:
        self._unity_data_store.store_received_data("calf_angle", msg.data)