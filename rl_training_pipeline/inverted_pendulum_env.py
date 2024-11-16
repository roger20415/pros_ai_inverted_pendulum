import gymnasium as gym
from gymnasium import spaces
import numpy as np
from utils import Utils
from config import Config

from subscribe_data.data_manager import DataManager

# TODO
class InvertedPendulumEnv(gym.Env):
    ENV_NAME: str = 'InvertedPendulum-v0'

    def __init__(self, data_manager: DataManager) -> None:
        super(InvertedPendulumEnv, self).__init__()
        
        self._data_manager = data_manager
        
        self._state_dict: dict[str, float] = self._data_manager.get_obervation()
        self._state_array: np.ndarray[np.float32] = Utils.flatten_dict_to_array(self._state_dict.copy)

        self._observation_shape: int = Utils.get_initial_shape(np.copy(self._state_array))
        
        self._observation_space = spaces.Box(
            low = -np.inf, high = np.inf, shape = self._observation_shape, dtype = np.float32
        )

        self._action_space = spaces.MultiDiscrete(Config.ACTION_NVEC)

    def step(self, action):

        reward: int = 1
        terminated: bool = False


        return self.state, reward, terminated, False, {}
    
    def reset(self, seed = None, options = None):
        return self.state, {}