import time

from config import Config

class LowFpsStopper:
    def __init__(self):
        self._low_fps_streak: int = 0
        self._should_reset_training: bool = False

    def calculate_low_fps_streak(self, fps: float) -> None:
        if fps < Config.LOW_FPS_THRESHOLD:
            self._low_fps_streak += 1
        else:
            self._low_fps_streak = 0

        print(f"FPS: {fps}, Low FPS Streak: {self._low_fps_streak}")
        self._check_and_stop_training(fps)
    
    def get_should_reset_training(self) -> bool:
        return self._should_reset_training
    
    def set_should_reset_training(self, should_reset_training: bool) -> None:
        self._should_reset_training = should_reset_training

    def _check_and_stop_training(self, fps: float) -> None:
        if self._low_fps_streak >= Config.MAX_LOW_FPS_STREAK:
            print(f"Training stopped due to low FPS: {fps}")
            self._low_fps_streak = -1
            self._should_reset_training = True
            time.sleep(Config.FPS_STOPPER_SLEEP_TIME)