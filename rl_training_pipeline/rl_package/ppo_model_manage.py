import os
import logging

from rclpy.node import Node
import gymnasium as gym
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import BaseCallback
from stable_baselines3.common.monitor import Monitor

from rl_package.inverted_pendulum_env import InvertedPendulumEnv
from config import Config
from subscribe_data.data_manage import DataManager
from publish_action.action_manage import ActionManager

class PPOModelManager:
    def register_gym_env(self, data_manager: DataManager, action_manager: ActionManager) -> gym.Env:
        
        gym.register(
            id = InvertedPendulumEnv.ENV_NAME,
            entry_point = "inverted_pendulum_env:InvertedPendulumEnv",
        )
        return gym.make(InvertedPendulumEnv.ENV_NAME, data_manager, action_manager)
    
    def train_model(self, env: gym.Env) -> None:
        model = self._load_or_create_model(env)
        check_point_callback = CheckPointCallback(Config.SAVE_MODEL_PATH, Config.SAVE_MODEL_FREQUENCY)
        model.learn(total_timesteps = Config.TRAINING_STEPS, 
                    callback = check_point_callback,
                    log_interval = Config.LOG_INTERVAL)
        
        return None
    
    def _load_or_create_model(self, env: gym.Env) -> PPO:
        
        env = Monitor(env)
        try:
            model = PPO.load(Config.LOAD_MODEL_PATH, env=env)
            print("Model loaded successfully!")
        except Exception as e:
            print(f"Error loading model: {e}. Creating a new model...")
            model = PPO("MlpPolicy", 
                        env, verbose=1, learning_rate=Config.LEARNING_RATE,
                        n_steps=Config.N_STEPS, batch_size=Config.BATCH_SIZE,
                        n_epochs=Config.N_EPOCHS, device="cuda")

        model.set_env(env)
        
        return model
        
        
class CheckPointCallback(BaseCallback):
    def __init__(self, save_path: str, save_freq: int, verbose: int = 0) -> None:
        super(CheckPointCallback, self).__init__(verbose)
        
        self.logger = logging.getLogger(__name__)
        logging.basicConfig(level = logging.INFO)
        
        self._save_path: str = save_path
        self._save_freq = save_freq
        self._last_save_step = 0
        
        return None
    
    def _on_step(self) -> bool:

        if self.n_calls - self._last_save_step >= self._save_freq:
            
            save_dir = os.path.dirname(self._save_path) 
            if not os.path.exists(save_dir):
                os.makedirs(save_dir)
                self.logger.info(f"Created new directory: {save_dir}")
            
            self.logger.info(f"Run {self.n_calls} steps...")
            self.model.save(self._save_path)
            self.logger.info(f"Model saved to {self._save_path}")
            self._last_save_step = self.n_calls

        return True   