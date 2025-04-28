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
        self.action_publisher.publish_target_joint_angles(target_joint_angles)
        
    def get_action_publisher_node(self) -> Node:
        return self.action_publisher
    
    def _quantize_action_to_servo_steps(self, action: np.ndarray) -> np.ndarray:
        # if action < 0.5 and action > -0.5:
        #     return np.array([0])
        # elif action > 0.1:
        #     return np.array([Config.SERVO_STEP_ANGLE])
        # elif action < -0.1:
        #     return np.array([-Config.SERVO_STEP_ANGLE])
        
        return np.round(action / Config.SERVO_STEP_ANGLE) * Config.SERVO_STEP_ANGLE

    def _add_action_noise(self, action: np.ndarray) -> np.ndarray:
        noisy_action: list[float] = []

        for joint_action in action:
            noise = random.uniform(-Config.ACTION_NOISE_LEVEL, Config.ACTION_NOISE_LEVEL)
            noisy_action.append(joint_action + noise)
            print(f"Original: {joint_action}, Noise: {noise}, Noisy Action: {noisy_action[-1]}")

        return np.array(noisy_action)
