import math

import numpy as np

from config import Config

class RewardCalculator:

    def __init__(self) -> None:
        self._previous_center_of_mass: float = 0.0

    def calculate_reward(self, state_dict: dict[str, float], action: np.ndarray, step_counter: int) -> float:

        foundation_angle: float = state_dict["foundation_angle"]
        center_of_mass: float = state_dict["center_of_mass"]

        print("action: ", action)
        if action == [0]:
            print("NO MOVE")
        elif action == [1]:
            print("+" + str(Config.JOINT_DELTA_UNIT))
        elif action == [2]:
            print("+" + str(2 * Config.JOINT_DELTA_UNIT))
        elif action == [3]:
            print("+" + str(3 * Config.JOINT_DELTA_UNIT))
        elif action == [4]:
            print("+" + str(4 * Config.JOINT_DELTA_UNIT))
        elif action == [5]:
            print("+" + str(5 * Config.JOINT_DELTA_UNIT))
        elif action == [6]:
            print("-" + str(Config.JOINT_DELTA_UNIT))
        elif action == [7]:
            print("-" + str(2 * Config.JOINT_DELTA_UNIT))
        elif action == [8]:
            print("-" + str(3 * Config.JOINT_DELTA_UNIT))
        elif action == [9]:
            print("-" + str(4 * Config.JOINT_DELTA_UNIT))
        elif action == [10]:
            print("-" + str(5 * Config.JOINT_DELTA_UNIT))
        print("calf angle", state_dict["calf_angle"])
        print("foundation angle", state_dict["foundation_angle"])
        print("center of mass", state_dict["center_of_mass"])
        #print("\n")

        #swing_alignment_reward: float = self._calculate_swing_alignment_reward(action)
        center_reward: float = self._calculate_center_of_mass_reward(center_of_mass)
        stability_bonus: float = self._calculate_stability_bonus(foundation_angle)
        tilt_penalty: float = self._calculate_tilt_penalty(foundation_angle)
        step_duration_reward: float = self._calculate_step_duration_reward(step_counter)
        reward: float = center_reward + stability_bonus + tilt_penalty + step_duration_reward
        self._previous_center_of_mass = center_of_mass

        #print("center reward: ", center_reward)
        #print("stability_bonus: ", stability_bonus)
        #print("tilt_penalty: ", tilt_penalty)
        #print("step_duration_reward: ", step_duration_reward)
        #print(f"reward: {reward}")
        print("--\n\n")

        if step_counter > 55:
            exit()

        return reward
    
    def reset_previous_center_of_mass(self) -> None:
        self._previous_center_of_mass = 0.0
    
    def _calculate_swing_alignment_reward(self, action: np.ndarray) -> float:

        swing_alignment_reward: float = -Config.SWING_ALIGNMENT_REWARD
        action_value: int = action[0]

        if ((self._previous_center_of_mass > 0.0 and action_value in Config.NEGATIVE_ACTIONS) or
            (self._previous_center_of_mass < 0.0 and action_value in Config.POSITIVE_ACTIONS) or 
            (self._previous_center_of_mass == 0.0 and action_value == Config.NO_MOVE_ACTION)):
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

    def _calculate_step_duration_reward(self, step_count: int) -> float:
            
        reward = math.exp(Config.STEP_DURATION_GROWTH_RATE * (step_count - Config.STEP_COUNT_THRESHOLD))

        # avoid overflow
        if Config.STEP_DURATION_GROWTH_RATE * (step_count - Config.STEP_COUNT_THRESHOLD) > 700:
            reward = math.exp(700)
        
        return reward