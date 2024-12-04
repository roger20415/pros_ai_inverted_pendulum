from config import Config

import numpy as np

class RewardCalculator:

    def __init__(self) -> None:
        self._previous_center_of_mass: float = 0.0

    def calculate_reward(self, state_dict: dict[str, float], action: np.ndarray) -> float:

        foundation_angle: float = state_dict["foundation_angle"]
        center_of_mass: float = state_dict["center_of_mass"]

        swing_alignment_reward: float = self._calculate_swing_alignment_reward(action)
        center_reward: float = self._calculate_center_of_mass_reward(center_of_mass)
        stability_bonus: float = self._calculate_stability_bonus(foundation_angle)
        tilt_penalty: float = self._calculate_tilt_penalty(foundation_angle)
        reward: float = swing_alignment_reward + center_reward + stability_bonus + tilt_penalty

        self._previous_center_of_mass = center_of_mass
        print(f"reward: {reward}")

        return reward
    
    def reset_previous_center_of_mass(self) -> None:
        self._previous_center_of_mass = 0.0
    
    def _calculate_swing_alignment_reward(self, action: np.ndarray) -> float:

        swing_alignment_reward: float = -Config.SWING_ALIGNMENT_REWARD
        action_value: int = action[0]

        if ((self._previous_center_of_mass > 0.0 and action_value in [6, 7, 8, 9, 10]) or
            (self._previous_center_of_mass <= 0.0 and action_value in [1, 2, 3, 4, 5]) or 
            (self._previous_center_of_mass == 0.0 and action_value == 0)):
            swing_alignment_reward = Config.SWING_ALIGNMENT_REWARD

        return swing_alignment_reward

    def _calculate_center_of_mass_reward(self, center_of_mass: float) -> float:
        center_change = abs(center_of_mass) - abs(self._previous_center_of_mass)
        
        return center_change * Config.CENTER_REWARD_WEIGHT

    def _calculate_stability_bonus(self, foundation_angle: float) -> float:
        stability_bonus: float = 0.0
        if abs(foundation_angle) < Config.STABILITY_BONUS_THRESHOLD:
            stability_bonus = Config.STABILITY_BONUS

        return stability_bonus
    
    def _calculate_tilt_penalty(self, foundation_angle: float) -> float:
        tilt_penalty: float = 0.0
        if abs(foundation_angle) > Config.TILT_PENALTY_THRESHOLD:
            tilt_penalty = Config.TILT_PENALTY

        return tilt_penalty
