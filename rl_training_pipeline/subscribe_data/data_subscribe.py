from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from subscribe_data.unity_data_store import UnityDataStore

class DataSubscriberNode(Node):
    def __init__(self, unity_data_store: UnityDataStore) -> None:
        super().__init__("inverted_pendulum_data_subscriber_node")
        self.get_logger().info("Start inverted pendulum data subscriber node.")

        self._unity_data_store = unity_data_store

        self._current_joint_angles_subscriber = self.create_subscription(
            Float32MultiArray,
            "/inverted_pendulum_current_joint_angles",
            self._current_joint_angles_subscribe_callback,
            1
        )

        self._joint_center_of_mass_subscriber = self.create_subscription(
            Float32MultiArray,
            "/inverted_pendulum_joint_center_of_mass",
            self._joint_center_of_mass_subscribe_callback,
            1
        )
        return None

    def _current_joint_angles_subscribe_callback(self, msg: Float32MultiArray) -> None:
        self._unity_data_store.store_received_data("current_joint_angles", msg.data)
        
        return None
    
    def _joint_center_of_mass_subscribe_callback(self, msg: Float32MultiArray) -> None:
        self._unity_data_store.store_received_data("joint_center_of_mass", msg.data)
        
        return None