import sys
import math

from numpy import flip

from config import Config

class RewardCalculator:

    def __init__(self) -> None:
        self._pre_calf_angle: float = 0.0

    def calculate_reward(self, state_dict: dict[str, float], step_counter: int) -> float:
        
        calf_angle: float = state_dict[Config.CALF_ANGLE_KEY]
        stable_reward: float = self._cal_stable_reward(step_counter)
        delta_calf_angle_reward: float = self._cal_delta_calf_angle_reward(calf_angle)
        flip_bonus: float = self._cal_flip_bonus(calf_angle)
        reward: float = stable_reward + delta_calf_angle_reward + flip_bonus
        sys.stderr.write(f"stable_reward: {stable_reward}\n")
        sys.stderr.write(f"calf_angle_reward: {delta_calf_angle_reward}\n")
        sys.stderr.write(f"flip_bonus: {flip_bonus}\n")

        self._pre_calf_angle = calf_angle
        return reward
    
    def reset_pre_calf_angle(self) -> None:
        self._pre_calf_angle = 0.0
    
    def _cal_stable_reward(self, step_counter: int) -> float:
        return step_counter*Config.STABLE_REWARD_WEIGHT

    def _cal_delta_calf_angle_reward(self, calf_angle: float) -> float:
        projection_calf_angle: float = abs(self._map_angle_to_x(calf_angle))
        projection_pre_calf_angle: float = abs(self._map_angle_to_x(self._pre_calf_angle))
        delta_calf_angle_reward: float = ((projection_calf_angle - projection_pre_calf_angle)
                                        *Config.DELTA_CALF_ANGLE_REWARD_WEIGHT)
        return delta_calf_angle_reward
    
    def _cal_flip_bonus(self, calf_angle: float) -> float:
        flip_bonus: float = 0.0
        if math.copysign(1, calf_angle) != math.copysign(1, self._pre_calf_angle):
            flip_bonus = Config.FLIP_BONUS                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             
        return flip_bonus

    def _map_angle_to_x(self, theta):
        return math.sin(math.radians(theta))