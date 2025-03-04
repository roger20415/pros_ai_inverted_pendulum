import sys

from config import Config

class RewardCalculator:

    def __init__(self) -> None:
        self._pre_calf_angle: float = 0.0

    def calculate_reward(self, state_dict: dict[str, float], step_counter: int) -> float:
        
        calf_angle: float = state_dict[Config.CALF_ANGLE_KEY]
        stable_reward: float = self._cal_stable_reward(step_counter)
        delta_calf_angle_reward: float = self._cal_delta_calf_angle_reward(calf_angle)
        reward: float = stable_reward + delta_calf_angle_reward
        sys.stderr.write(f"stable_reward: {stable_reward}\n")
        sys.stderr.write(f"calf_angle_reward: {delta_calf_angle_reward}\n")

        self._pre_calf_angle = calf_angle
        return reward
    
    def reset_pre_calf_angle(self) -> None:
        self._pre_calf_angle = 0.0
    
    def _cal_stable_reward(self, step_counter: int) -> float:
        return step_counter*Config.STABLE_REWARD_WEIGHT

    def _cal_delta_calf_angle_reward(self, calf_angle: float) -> float:
        calf_angle: float = abs(calf_angle)
        pre_calf_angle: float = abs(self._pre_calf_angle)
        
        delta_calf_angle_reward: float = ((calf_angle - pre_calf_angle)
                                        *Config.DELTA_CALF_ANGLE_REWARD_WEIGHT)
        return delta_calf_angle_reward