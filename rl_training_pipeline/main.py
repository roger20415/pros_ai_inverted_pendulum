from user_cli import UserCLI
import gymnasium as gym
from inverted_pendulum_env import InvertedPendulumEnv
from config import Config, ValidMode

class MainProcessor:
    def __init__(self):
        self.user_cli = UserCLI()

    def _register_gym_env(self) -> gym.Env:
        gym.register(
            id = InvertedPendulumEnv.ENV_NAME,
            entry_point = "inverted_pendulum_env:InvertedPendulumEnv",
        )
        return gym.make(InvertedPendulumEnv.ENV_NAME)
    
    def _train_model(self, env: gym.Env) -> None:
        pass

if __name__ == "__main__":
    main_processor = MainProcessor()
    
    main_processor.user_cli.execute_user_cli()
    user_input_mode: str = main_processor.user_cli.get_user_input_mode()

    env: gym.Env =  main_processor._register_gym_env()

    if user_input_mode == ValidMode.TRAIN.value:
        main_processor._train_model(env)

