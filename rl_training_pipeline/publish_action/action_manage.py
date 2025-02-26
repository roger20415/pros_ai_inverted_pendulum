import numpy as np
from rclpy.node import Node

from publish_action.action_publish import ActionPublisherNode
from config import Config

class ActionManager:
    def __init__(self) -> None:
        self.action_publisher = ActionPublisherNode()
        
    def process_and_publish_actions(self, action: np.ndarray) -> None:
        action = self._quantize_action_to_servo_steps(action)
        target_joint_angles: list[float] = action.tolist()
        self.action_publisher.publish_target_joint_angles(target_joint_angles)
        
    def get_action_publisher_node(self) -> Node:
        return self.action_publisher
    
    def _quantize_action_to_servo_steps(self, action: np.ndarray) -> np.ndarray:
        return np.round(action / Config.SERVO_STEP_ANGLE) * Config.SERVO_STEP_ANGLE