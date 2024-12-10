from utils import Utils

from config import Config

class WaitingDataMonitor:
    def __init__(self) -> None:
        self._waiting_time_list: list[int] = []

    def append_waiting_time(self, waiting_time: int) -> None:
        self._waiting_time_list.append(waiting_time)
        self._waiting_unity_too_long_stopper(waiting_time)

    def save_waiting_time_plot(self, save_path: str) -> None:
        Utils.save_plot(
            xdata=list(range(len(self._waiting_time_list))),
            ydata=self._waiting_time_list,
            title="Waiting for Data time",
            xlabel="Steps",
            ylabel="Waiting time (µs)",
            save_path=save_path
        )

    def _waiting_unity_too_long_stopper(self, waiting_time: int) -> None:
        if waiting_time > Config.WAITING_TIME_STOP_THRESHOLD:
            raise Exception("Waiting for Unity data took too long. Stop training.")