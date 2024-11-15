import gymnasium as gym
from gymnasium import spaces
import numpy as np
from utils import Utils
from config import Config


# TODO
class InvertedPendulumEnv(gym.Env):
    ENV_NAME: str = 'InvertedPendulum-v0'

    def __init__(self) -> None:
        super(InvertedPendulumEnv, self).__init__()

        self._observation_shape = Utils.get_initial_shape()
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