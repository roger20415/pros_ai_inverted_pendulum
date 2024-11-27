from utils import Utils

class WaitingDataMonitor:
    def __init__(self) -> None:
        self._waiting_time_list: list[int] = []

    def append_waiting_time(self, waiting_time: int) -> None:
        self._waiting_time_list.append(waiting_time)

    def save_waiting_time_plot(self, save_path: str) -> None:
        Utils.save_plot(
            xdata=list(range(len(self._waiting_time_list))),
            ydata=self._waiting_time_list,
            title="Waiting for Data time",
            xlabel="Steps",
            ylabel="Waiting time (µs)",
            save_path=save_path
        )