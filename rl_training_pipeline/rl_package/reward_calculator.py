from config import Config

class RewardCalculator:

    def __init__(self) -> None:
        self._previous_foundation_angle: float = 0.0

    def calculate_reward(self, state_dict: dict[str, float]) -> float:
    
        foundation_angle: float = abs(state_dict["foundation_angle"])

        angle_penalty: float = self._calculate_angle_reward(foundation_angle)
        stability_bonus: float = self._calculate_stability_bonus(foundation_angle)
        tilt_penalty: float = self._calculate_tilt_penalty(foundation_angle)
        reward: float = angle_penalty + stability_bonus + tilt_penalty

        self._previous_foundation_angle = foundation_angle
        print(f"reward: {reward}")

        return reward
    
    def reset_previous_foundation_angle(self) -> None:
        self._previous_foundation_angle = 0.0
    
    def _calculate_angle_reward(self, foundation_angle: float) -> float:
        angle_change = foundation_angle - self._previous_foundation_angle
        
        return angle_change * Config.ANGLE_REWARD_WEIGHT
    
    def _calculate_stability_bonus(self, foundation_angle: float) -> float:
        stability_bonus: float = 0.0
        if foundation_angle < Config.STABILITY_BONUS_THRESHOLD:
            stability_bonus = Config.STABILITY_BONUS

        return stability_bonus
    
    def _calculate_tilt_penalty(self, foundation_angle: float) -> float:
        tilt_penalty: float = 0.0
        if foundation_angle > Config.TILT_PENALTY_THRESHOLD:
            tilt_penalty = Config.TILT_PENALTY

        return tilt_penalty
