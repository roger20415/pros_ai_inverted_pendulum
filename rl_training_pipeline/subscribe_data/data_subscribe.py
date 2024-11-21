from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from subscribe_data.unity_data_store import UnityDataStore

class DataSubscriberNode(Node):
    def __init__(self, unity_data_store: UnityDataStore) -> None:
        super().__init__("inverted_pendulum_data_subscriber_node")
        self.get_logger().info("Start inverted pendulum data subscriber node.")

        self._unity_data_store = unity_data_store

        self._calf_angle_subscriber = self.create_subscription(
            Float32MultiArray,
            "/inverted_pendulum_calf_angle",
            lambda msg: self._subscribe_callback("calf_angle", msg),
            1
        )

        self._foundation_angle_subscriber = self.create_subscription(
            Float32MultiArray,
            "/inverted_pendulum_foundation_angle",
            lambda msg: self._subscribe_callback("foundation_angle", msg),
            1
        )

        self._baselink_center_of_mass_subscriber = self.create_subscription(
            Float32MultiArray,
            "/inverted_pendulum_baselink_center_of_mass",
            lambda msg: self._subscribe_callback("baselink_center_of_mass", msg),
            1
        )
    
        self._calf_center_of_mass_subscriber = self.create_subscription(
            Float32MultiArray,
            "/inverted_pendulum_calf_center_of_mass",
            lambda msg: self._subscribe_callback("calf_center_of_mass", msg),
            1
        )
        

    def _subscribe_callback(self, key: str, msg: Float32MultiArray) -> None:
        self._unity_data_store.store_received_data(key, msg)
        