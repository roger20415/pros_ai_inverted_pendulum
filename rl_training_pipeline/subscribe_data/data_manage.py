from rclpy.node import Node

from subscribe_data.data_transform import DataTransformer
from subscribe_data.data_subscribe import DataSubscriberNode
from subscribe_data.unity_data_store import UnityDataStore
from subscribe_data.waiting_data_monitor import WaitingDataMonitor
from subscribe_data.com_monitor import ComMonitor

class DataManager:
    def __init__(self) -> None:
        self.unity_data_store = UnityDataStore()
        self.data_transformer = DataTransformer(self.unity_data_store)
        self.data_subscriber = DataSubscriberNode(self.unity_data_store)
        self.waiting_data_monitor = WaitingDataMonitor()
        self.com_monitor = ComMonitor()
        
    def get_obervation(self) -> dict[str, float]:
        
        self.unity_data_store.turn_all_data_flag_to_unready()
        self.unity_data_store.wait_all_data_ready(self.waiting_data_monitor)
        observation_state: dict[str, float] = self.data_transformer.transform_untiy_data_to_state()
        self.com_monitor.append_com(observation_state["center_of_mass"])
        
        return observation_state
    
    def get_data_subscriber_node(self) -> Node:
        return self.data_subscriber
    
    def reset(self) -> None:
        self.data_transformer.reset()
            