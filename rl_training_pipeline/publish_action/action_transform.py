import numpy as np

from config import Config

class ActionTransformer:
    def transfer_actions_to_target_joint_angles(self, actions: np.ndarray, current_joint_positions: list[float]) -> list[float]:
        
        joint_angle_deltas = self._convert_actions_to_joint_angle_deltas(actions)
        target_joint_angles = self._calculate_target_joint_angles(joint_angle_deltas, current_joint_positions)
        target_joint_angles = self._limit_target_joint_angles(target_joint_angles)
        
        return target_joint_angles
    
    def _convert_actions_to_joint_angle_deltas(self, actions: np.ndarray) -> list[float]:
        
        joint_angle_deltas: list[float] = []
        for action in actions:
            if action == Config.NO_MOVE_ACTION: #0
                joint_angle_deltas.append(0)     # +0
            elif action == Config.POSITIVE_ACTIONS[0]: #1
                joint_angle_deltas.append(Config.JOINT_DELTA_UNIT) # +4
            elif action == Config.POSITIVE_ACTIONS[1]: #2
                joint_angle_deltas.append(2 * Config.JOINT_DELTA_UNIT) # +8
            elif action == Config.POSITIVE_ACTIONS[2]: #3
                joint_angle_deltas.append(3 * Config.JOINT_DELTA_UNIT) # +12
            elif action == Config.POSITIVE_ACTIONS[3]: #4 
                joint_angle_deltas.append(4 * Config.JOINT_DELTA_UNIT) # +16
            elif action == Config.POSITIVE_ACTIONS[4]: #5
                joint_angle_deltas.append(5 * Config.JOINT_DELTA_UNIT) # +20
            elif action == Config.NEGATIVE_ACTIONS[0]: #6
                joint_angle_deltas.append(-Config.JOINT_DELTA_UNIT) # -4
            elif action == Config.NEGATIVE_ACTIONS[1]: #7
                joint_angle_deltas.append(-2 * Config.JOINT_DELTA_UNIT) # -8
            elif action == Config.NEGATIVE_ACTIONS[2]: #8
                joint_angle_deltas.append(-3 * Config.JOINT_DELTA_UNIT) # -12
            elif action == Config.NEGATIVE_ACTIONS[3]: #9
                joint_angle_deltas.append(-4 * Config.JOINT_DELTA_UNIT) # -16
            elif action == Config.NEGATIVE_ACTIONS[4]: #10
                joint_angle_deltas.append(-5 * Config.JOINT_DELTA_UNIT) # -20
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
            -Config.MAX_JOINT_ANGLE,
            Config.MAX_JOINT_ANGLE
        )
        
        target_joint_angles: list[float] = limited_target_joint_angles_array.tolist()
        
        return target_joint_angles