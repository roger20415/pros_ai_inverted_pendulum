import gymnasium as gym
from stable_baselines3 import PPO
from stable_baselines3.common.monitor import Monitor

from rl_package.inverted_pendulum_env import InvertedPendulumEnv
from config import Config
from unity_state.unity_state_communication import UnityStateManagerNode
from subscribe_data.data_manage import DataManager
from publish_action.action_manage import ActionManager
from rl_package.fps_plot_callback import FpsPlotCallback
from rl_package.save_model_callback import SaveModelCallback

class PPOModelManager:
    def register_gym_env(self, data_manager: DataManager, action_manager: ActionManager, unity_state_manager: UnityStateManagerNode) -> gym.Env:
        
        gym.register(
            id = InvertedPendulumEnv.ENV_NAME,
            entry_point = "rl_package.inverted_pendulum_env:InvertedPendulumEnv",
        )
        return gym.make(
            InvertedPendulumEnv.ENV_NAME, 
            data_manager=data_manager, 
            action_manager=action_manager, 
            unity_state_manager=unity_state_manager)
    
    def train_model(self, env: gym.Env) -> None:
        model = self._load_or_create_model(env)
        save_model_callback = SaveModelCallback(Config.SAVE_MODEL_PATH, Config.SAVE_MODEL_FREQUENCY)
        fps_plot_callback = FpsPlotCallback()
        callbacks = [save_model_callback, fps_plot_callback]
        model.learn(total_timesteps=Config.TRAINING_STEPS, 
                    callback=callbacks,
                    log_interval=Config.LOG_INTERVAL)
        fps_plot_callback.save_fps_plot()
        
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
                        n_epochs=Config.N_EPOCHS, device="cpu")
        print(f'Model is on device: {model.policy.device}')

        model.set_env(env)
        
        return model