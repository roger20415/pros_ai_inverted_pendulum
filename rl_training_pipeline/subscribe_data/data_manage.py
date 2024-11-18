from rclpy.node import Node

from subscribe_data.data_transform import DataTransformer
from subscribe_data.data_subscribe import DataSubscriberNode
from subscribe_data.unity_data_store import UnityDataStore

class DataManager:
    def __init__(self) -> None:
        self.unity_data_store = UnityDataStore()
        self.data_transformer = DataTransformer(self.unity_data_store)
        self.data_subscriber = DataSubscriberNode(self.unity_data_store)
        
        return None
        
    def get_obervation(self) -> dict[str, float]:
        
        self.unity_data_store.turn_all_data_flag_to_unready()
        self.unity_data_store.wait_all_data_ready()
        observation_state: dict[str, float] = self.data_transformer.transform_untiy_data_to_state()
        
        return observation_state
    
    def get_data_subscriber_node(self) -> Node:
        return self.data_subscriber
            