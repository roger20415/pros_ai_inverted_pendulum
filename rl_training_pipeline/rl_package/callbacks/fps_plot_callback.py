import time
import matplotlib.pyplot as plt
from stable_baselines3.common.callbacks import BaseCallback
from config import Config

class FpsPlotCallback(BaseCallback):
    def __init__(self, verbose=0) -> None:
        super(FpsPlotCallback, self).__init__(verbose)
        self._fps_list: list[float] = []
        self._steps: list[int] = []
        self._last_n_calls: int = 0
        self._interval_start_time: float = time.time()

    def save_fps_plot(self) -> None:
        plt.plot(self._steps, self._fps_list)
        plt.title("FPS Over Time")
        plt.xlabel("Steps")
        plt.ylabel("FPS")
        plt.savefig(Config.FPS_PLOT_PATH) 
        print("FPS plot saved as " + Config.FPS_PLOT_PATH)
    
    def on_training_end(self) -> None:
        self.save_fps_plot()
        return None
    
    def _on_step(self) -> bool:
        if self.n_calls % Config.FPS_LOG_INTERVAL == 0:
            interval_end_time: float = time.time()
            time_interval: float = interval_end_time - self._interval_start_time
            
            fps: float = (self.n_calls - self._last_n_calls) / time_interval
            self._fps_list.append(fps)
            self._steps.append(self.n_calls)
            self._last_n_calls = self.n_calls 
            self._interval_start_time = time.time()

        return True
