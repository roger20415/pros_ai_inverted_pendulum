import numpy as np
from action_transformer import ActionTransformer
from action_publisher import ActionPublisherNode
from rclpy.node import Node

class ActionManager:
    def __init__(self, action_transformer: ActionTransformer, action_publisher: ActionPublisherNode) -> None:
        self.action_transformer = action_transformer
        self.action_publisher = action_publisher
        
        return None
        
    def process_and_publish_actions(self, action: np.ndarray, current_joint_positions: list[float]) -> None:
        target_joint_angles: list[float] = self.action_transformer.transfer_actions_to_target_joint_angles(
            action, current_joint_positions)
        
        self.action_publisher.publish_target_joint_angles(target_joint_angles)
        
        return None
    
    def get_action_publisher_node(self) -> Node:
        return self.action_publisher