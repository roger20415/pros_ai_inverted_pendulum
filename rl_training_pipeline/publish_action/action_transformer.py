import numpy as np
import math
from config import Config
from utils import Utils

class ActionTransformer:
    
    def transfer_actions_to_target_joint_angles(self, actions: np.ndarray, current_joint_positions: list[float]) -> list[float]:
        
        joint_angle_deltas = self._convert_actions_to_joint_angle_deltas(actions)
        target_joint_angles = self._calculate_target_joint_angles(joint_angle_deltas, current_joint_positions)
        target_joint_angles = self._limit_target_joint_angles(target_joint_angles)
        target_joint_angles = Utils.radians_degrees_transfer(target_joint_angles, "degrees2radians")
        
        return target_joint_angles
    
    def _convert_actions_to_joint_angle_deltas(self, actions: np.ndarray) -> list[float]:
        
        joint_angle_deltas: list[float] = []
        for action in actions:
            if action == 0:
                joint_angle_deltas.append(0)
            elif action == 1:
                joint_angle_deltas.append(Config.JOINT_DELTA_UNIT)
            elif action == 2:
                joint_angle_deltas.append(-Config.JOINT_DELTA_UNIT)
            else:
                raise ValueError("Invalid action value: {}".format(action))
            
        return joint_angle_deltas
    
    def _calculate_target_joint_angles(self, joint_angle_deltas: list[float], current_joint_angles: list[float]) -> list[float]:
        
        return np.add(joint_angle_deltas, current_joint_angles).tolist()
    
    def _limit_target_joint_angles(self, target_joint_angles: list[float]) -> list[float]:
        """
        Limit the target joint angles' degrees.

        Parameters
        ----------
        target_joint_angles: list[float]
            The target joint angles, measured in degrees, are derived from the PPO model's actions.
        
        Returns
        ----------
        target_joint_angles: list[float]
            The joint target angles, measured in degrees, are constrained by their respective limits.
        """

        limited_target_joint_angles_array: np.ndarray = np.clip(
            target_joint_angles,
            math.radians(-Config.MAX_JOINT_ANGLE),
            math.radians(Config.MAX_JOINT_ANGLE)
        )
        
        target_joint_angles: list[float] = limited_target_joint_angles_array.tolist()
        
        return target_joint_angles