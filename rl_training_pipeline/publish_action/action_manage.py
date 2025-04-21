import numpy as np
import random
from rclpy.node import Node

from publish_action.action_publish import ActionPublisherNode
from config import Config

class ActionManager:
    def __init__(self) -> None:
        self.action_publisher = ActionPublisherNode()
        
    def process_and_publish_actions(self, action: np.ndarray) -> None:
        action = self._quantize_action_to_servo_steps(action)
        action = self._add_action_noise(action)
        target_joint_angles: list[float] = action.tolist()
        final_joint_angles: list[float] = self._gen_sym_angles(target_joint_angles)
        self.action_publisher.publish_target_joint_angles(final_joint_angles)
        
    def get_action_publisher_node(self) -> Node:
        return self.action_publisher
    
    def _quantize_action_to_servo_steps(self, action: np.ndarray) -> np.ndarray:
        return np.round(action / Config.SERVO_STEP_ANGLE) * Config.SERVO_STEP_ANGLE

    def _add_action_noise(self, action: np.ndarray) -> np.ndarray:
        noisy_action: list[float] = []

        for joint_action in action:
            noise = random.uniform(-Config.ACTION_NOISE_LEVEL, Config.ACTION_NOISE_LEVEL)
            noisy_action.append(joint_action + noise)

        return np.array(noisy_action)
    
    def _gen_sym_angles(self, target_joint_angles: list[float]) -> list[float]:
        sym_angles = []

        for angle in target_joint_angles:
            sym_angles.append(angle)
            sym_angles.append(-angle)

        return sym_angles
