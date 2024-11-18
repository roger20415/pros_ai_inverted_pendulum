import numpy as np
import gymnasium as gym
from gymnasium import spaces

from publish_action.action_manage import ActionManager
from subscribe_data.data_manage import DataManager
from utils import Utils
from config import Config

class InvertedPendulumEnv(gym.Env):
    ENV_NAME: str = 'InvertedPendulum-v0'

    def __init__(self, data_manager: DataManager, action_manager: ActionManager) -> None:
        super(InvertedPendulumEnv, self).__init__()

        self._data_manager = data_manager
        self._action_manager = action_manager
        
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
        self._action_manager.process_and_publish_actions(action, [self._state_dict.get("calf_angle", 0)])
        self._update_state()
        # TODO: Compute reward and termination conditions
        reward: int = 0
        terminated: bool = False

        return self._state_array, reward, terminated, False, {}

    def reset(self, seed = None, options = None):
        self._update_state()
        
        return self._state_array, {}
    
    def _update_state(self) -> None:
        self._state_dict = self._data_manager.get_obervation()
        self._state_array = Utils.flatten_dict_to_array(self._state_dict.copy())
        
        return None