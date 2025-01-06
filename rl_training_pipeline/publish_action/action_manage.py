import numpy as np
from rclpy.node import Node

from publish_action.action_transform import ActionTransformer
from publish_action.action_publish import ActionPublisherNode

class ActionManager:
    def __init__(self) -> None:
        self.action_transformer = ActionTransformer()
        self.action_publisher = ActionPublisherNode()
        
    def process_and_publish_actions(self, action: np.ndarray, current_joint_positions: list[float]) -> None:
        target_joint_angles: list[float] = self.action_transformer.transfer_actions_to_target_joint_angles(
            action, current_joint_positions)
        self.action_publisher.publish_target_joint_angles(target_joint_angles)
        
    def get_action_publisher_node(self) -> Node:
        return self.action_publisher