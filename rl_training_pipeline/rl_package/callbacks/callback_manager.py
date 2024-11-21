from stable_baselines3.common.callbacks import BaseCallback

from rl_package.callbacks.fps_plot_callback import FpsPlotCallback
from rl_package.callbacks.save_model_callback import SaveModelCallback
from config import Config

class CallbackManager:
        
    def __init__(self) -> None:
        self.save_model_callback = SaveModelCallback(Config.SAVE_MODEL_PATH, Config.SAVE_MODEL_FREQUENCY)
        self.fps_plot_callback = FpsPlotCallback()

        self.callbacks: list[BaseCallback] = [self.save_model_callback, self.fps_plot_callback]

        return None
    
    def get_callbacks(self) -> list[BaseCallback]:
        return self.callbacks