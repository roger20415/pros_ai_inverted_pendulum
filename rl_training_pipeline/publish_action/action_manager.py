import numpy as np
from threading import Thread
from action_transformer import ActionTransformer
from action_publisher import ActionPublisherNode
from rclpy.node import Node

class ActionManager:
    def __init__(self, action_transformer: ActionTransformer, action_publisher: ActionPublisherNode) -> None:
        self.action_transformer = action_transformer
        self.action_publisher = action_publisher
        
        
    def publish_action(self, action: np.ndarray, current_joint_positions: list[float]) -> None:
        joints_targets: list[float] = self.action_transformer.transform_action_to_joint_targets(
            action, current_joint_positions)
        
        self.action_publisher.publish_joint_targets(joints_targets)
        
        return None
    
    def get_action_publisher_node(self) -> Node:
        return self.action_publisher