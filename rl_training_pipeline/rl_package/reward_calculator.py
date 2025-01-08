import numpy as np

from config import Config

class RewardCalculator:

    def __init__(self) -> None:
        self._previous_center_of_mass: float = 0.0

    def calculate_reward(self, state_dict: dict[str, float], step_counter: int) -> float:

        center_of_mass: float = state_dict[Config.CENTER_OF_MASS_KEY]
        calf_angle: float = state_dict[Config.CALF_ANGLE_KEY]

        reward: float = 0.0
        reward += self._cal_stable_reward(step_counter)
        print("stable reward:", reward)

        reward += self._cal_delta_center_of_mass_reward(center_of_mass)
        print("delta center of mass reward:", reward)

        self._previous_center_of_mass = center_of_mass

        return reward
    
    def reset_previous_center_of_mass(self) -> None:
        self._previous_center_of_mass = 0.0
    
    def _cal_stable_reward(self, step_counter: int) -> float:
        return step_counter*Config.STABLE_REWARD_WEIGHT

    def _cal_delta_center_of_mass_reward(self, center_of_mass: float) -> float:
        center_of_mass: float = abs(center_of_mass)
        pre_center_of_mass: float = abs(self._previous_center_of_mass)
        
        delta_center_of_mass_reward: float = ((center_of_mass - pre_center_of_mass)
                                        *Config.CENTER_OF_MASS_REWARD_WEIGHT)
        return delta_center_of_mass_reward