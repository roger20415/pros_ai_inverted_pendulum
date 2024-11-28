from config import Config

class LowFpsStopper:
    def __init__(self):
        self._low_fps_streak: int = 0

    def calculate_low_fps_streak(self, fps: float) -> None:
        if fps < Config.LOW_FPS_THRESHOLD:
            self._low_fps_streak += 1
        else:
            self._low_fps_streak = 0

        print(f"FPS: {fps}, Low FPS Streak: {self._low_fps_streak}")
        self._check_and_stop_training(fps)

    def _check_and_stop_training(self, fps: float) -> None:
        if self._low_fps_streak >= Config.MAX_LOW_FPS_STREAK:
            print(f"Training stopped due to low FPS: {fps}")
            exit(0)