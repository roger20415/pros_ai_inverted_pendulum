import time

import numpy as np
import gymnasium as gym
from gymnasium import spaces

from rl_package.duration_step_monitor import DurationStepMonitor
from rl_package.reward_monitor import RewardMonitor
from publish_action.action_manage import ActionManager
from subscribe_data.data_manage import DataManager
from rl_package.reward_calculator import RewardCalculator
from utils import Utils
from config import Config

class InvertedPendulumEnv(gym.Env):
    ENV_NAME: str = 'InvertedPendulum-v0'

    def __init__(self, data_manager: DataManager, action_manager: ActionManager) -> None:
        super(InvertedPendulumEnv, self).__init__()
        self.data_manager = data_manager
        self.action_manager = action_manager
        self.reward_calculator = RewardCalculator()
        self.reward_monitor = RewardMonitor()
        self.duration_steps_monitor = DurationStepMonitor()
        
        self._state_dict: dict[str, float] = {}
        self._state_array: np.ndarray[np.float32] = np.array([])

        self._step_counter: int = 0
        self._total_step_counter: int = 0

        self.data_manager.unity_data_store.record_pub_servo_angle(0)
        self._update_state()
        self._observation_shape: int = len(self._state_array)
        
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(self._observation_shape,), dtype=np.float32
        )
        self.action_space = spaces.Box(low=-Config.MAX_JOINT_ANGLE, 
                                       high=Config.MAX_JOINT_ANGLE, 
                                       shape=(Config.ACTION_NVEC,), 
                                       dtype=np.float32)
        
    def step(self, action):
        self._trigger_reset_if_updated_params()
        action_pub: np.ndarray = self.action_manager.process_and_publish_actions(action)
        self.data_manager.unity_data_store.record_pub_servo_angle(action_pub[0])
        self._update_state()
        for key, value in self._state_dict.items():
            print(f"{key}, {value}")
        reward: float = self.reward_calculator.calculate_reward(self._state_dict, self._step_counter)
        if self._total_step_counter % Config.N_STEPS == 0:
            reward = 0.0
        terminated: bool = self._should_terminate(self._state_dict)
        self.reward_monitor.add_reward(reward)
        self.duration_steps_monitor.add_duration_steps()
        self._step_counter += 1
        self._total_step_counter += 1

        return self._state_array, reward, terminated, False, {}

    def reset(self, seed=None, options=None):
        self.duration_steps_monitor.append_duration_steps_to_list()
        self.duration_steps_monitor.save_duration_steps_plot(Config.DURATION_STEPS_PLOT_PATH)
        self.duration_steps_monitor.save_avg_duration_steps_plot(Config.AVERATE_DURATION_STEPS_PLOT_PATH)
        
        self.reward_monitor.save_reward_plot(Config.REWARD_PLOT_PATH)
        self.reward_monitor.save_avg_reward_plot(Config.AVERAGE_REWARD_PLOT_PATH)

        self._step_counter = 0
        self.data_manager.reset()

        print("\n-----------reset-------------\n")
        time.sleep(1.5)
        self.reward_calculator.reset_pre_calf_angle()
        self._update_state()

        return self._state_array, {}
    
    def _update_state(self) -> None:
        self._state_dict = self.data_manager.get_obervation()
        self._state_array = Utils.flatten_dict_to_array(self._state_dict.copy())
        
    def _should_terminate(self, state: dict[str, float]) -> bool:
        terminated: bool = False
        calf_angle: float = abs(state[Config.CALF_ANGLE_KEY])
        
        if calf_angle > Config.TERMINATE_THRESHOLD:
            terminated = True
        return terminated

    def _trigger_reset_if_updated_params(self) -> None:
        if (self._total_step_counter % Config.N_STEPS == 0) and (self._total_step_counter != 0):
            self.reset()