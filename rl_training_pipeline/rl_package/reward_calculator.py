import sys
import math

from numpy import flip

from config import Config

class RewardCalculator:

    def __init__(self) -> None:
        self._pre_foundation_angle: float = 0.0

    def calculate_reward(self, state_dict: dict[str, float], step_counter: int) -> float:
        
        foundation_angle: float = state_dict[Config.FOUNDATION_ANGLE_KEY]
        stable_reward: float = self._cal_stable_reward(step_counter)
        delta_foundation_angle_reward: float = self._cal_delta_foundation_angle_reward(foundation_angle)
        flip_bonus: float = self._cal_flip_bonus(foundation_angle)
        reward: float = stable_reward + delta_foundation_angle_reward + flip_bonus
        sys.stderr.write(f"stable_reward: {stable_reward}\n")
        sys.stderr.write(f"foundation_angle_reward: {delta_foundation_angle_reward}\n")
        sys.stderr.write(f"flip_bonus: {flip_bonus}\n")

        self._pre_foundation_angle = foundation_angle
        return reward
    
    def reset_pre_foundation_angle(self) -> None:
        self._pre_foundation_angle = 0.0
    
    def _cal_stable_reward(self, step_counter: int) -> float:
        return step_counter*Config.STABLE_REWARD_WEIGHT

    def _cal_delta_foundation_angle_reward(self, foundation_angle: float) -> float:
        projection_foundation_angle: float = abs(self._map_angle_to_x(foundation_angle))
        projection_pre_foundation_angle: float = abs(self._map_angle_to_x(self._pre_foundation_angle))
        delta_foundation_angle_reward: float = ((projection_foundation_angle - projection_pre_foundation_angle)
                                        *Config.DELTA_foundation_angle_REWARD_WEIGHT)
        return delta_foundation_angle_reward
    
    def _cal_flip_bonus(self, foundation_angle: float) -> float:
        flip_bonus: float = 0.0
        if math.copysign(1, foundation_angle) != math.copysign(1, self._pre_foundation_angle):
            flip_bonus = Config.FLIP_BONUS                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             
        return flip_bonus

    def _map_angle_to_x(self, theta):
        return math.sin(math.radians(theta))