import numpy as np
import gymnasium as gym
from gymnasium import spaces

from publish_action.action_manage import ActionManager
from subscribe_data.data_manage import DataManager
from unity_state.unity_state_communication import UnityStateManagerNode
from rl_package.reward_calculator import RewardCalculator
from utils import Utils
from config import Config

class InvertedPendulumEnv(gym.Env):
    ENV_NAME: str = 'InvertedPendulum-v0'

    def __init__(self, data_manager: DataManager, action_manager: ActionManager, unity_state_manager: UnityStateManagerNode) -> None:
        super(InvertedPendulumEnv, self).__init__()

        self.data_manager = data_manager
        self.action_manager = action_manager
        self.unity_state_manager = unity_state_manager
        self.reward_calculator = RewardCalculator()
        
        self._state_dict: dict[str, float] = {}
        self._state_array: np.ndarray[np.float32] = np.array([])

        self._update_state()
        self._observation_shape: int = len(self._state_array)
        
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(self._observation_shape,), dtype=np.float32
        )
        self.action_space = spaces.MultiDiscrete(Config.ACTION_NVEC)
        return None

    def step(self, action):
        self.action_manager.process_and_publish_actions(action, [self._state_dict.get("calf_angle", 0)])
        self._update_state()
        reward: float = self.reward_calculator.calculate_reward(self._state_dict)
        terminated: bool = self._should_terminate(self._state_dict)

        return self._state_array, reward, terminated, False, {}

    def reset(self, seed=None, options=None):
        self.unity_state_manager.set_is_training_paused(True)
        self.unity_state_manager.publish_reset_unity_scene(True)

        i: int = 0 # counter
        while(self.unity_state_manager.get_is_training_paused()):
            i += 1
            if i % 10000 == 0:
                print("Unity scene reset failed. Try again...")
                self.unity_state_manager.get_is_training_paused()
            
        self._update_state()
        
        return self._state_array, {}
    
    def _update_state(self) -> None:
        self._state_dict = self.data_manager.get_obervation()
        self._state_array = Utils.flatten_dict_to_array(self._state_dict.copy())
        
        return None
    
    def _should_terminate(self, state: dict[str, float]) -> bool:
        terminated: bool = False
        foundation_angle: float = abs(state["foundation_angle"])
        
        if foundation_angle > Config.TERMINATE_THRESHOLD:
            terminated = True
        
        return terminated