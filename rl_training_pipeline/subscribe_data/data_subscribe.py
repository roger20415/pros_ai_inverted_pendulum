import sys
from datetime import datetime

from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from subscribe_data.unity_data_store import UnityDataStore

class DataSubscriberNode(Node):
    def __init__(self, unity_data_store: UnityDataStore) -> None:
        super().__init__("inverted_pendulum_data_subscriber_node")
        self.get_logger().info("Start inverted pendulum data subscriber node.")

        self._last_pub_time: float = 0.0
        self._unity_data_store = unity_data_store

        self._state_subscriber = self.create_subscription(
            Float32MultiArray,
            "/state_topic",
            self._state_subscribe_callback,
            10
        )

    def _state_subscribe_callback(self, msg: Float32MultiArray) -> None:
        now = datetime.now()
        now_seconds = now.second + now.microsecond / 1_000_000
        self._unity_data_store.split_and_store_received_array(msg)
        sys.stderr.write(f"\033[35m{msg.data}\033[0m")
        self._print_pub_sub_info(
            msg.data[4],
            msg.data[5],
            now_seconds
        )
        self._last_pub_time = msg.data[5]


    def _print_pub_sub_info(self, fixupdate_count: float, pub_time: float, sub_time: float) -> None:
        sys.stderr.write("\033[92mState: {:.0f}\033[0m\n".format(fixupdate_count))
        sys.stderr.write("\033[92mPub time: {:.4f}s\033[0m\n".format(pub_time))
        sys.stderr.write("\033[32mSub time: {:.4f}s\033[0m\n".format(sub_time))
        sys.stderr.write("\033[92m2 Pub Interval: {:.4f}s\033[0m\n".format(pub_time - self._last_pub_time))
        sys.stderr.write("\033[92mPub-Sub Interval: {:.4f}s\n\033[0m\n".format(sub_time - pub_time))
