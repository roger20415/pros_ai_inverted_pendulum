from utils import Utils

class WaitingDataMonitor:
    def __init__(self) -> None:
        self._waiting_count_list: list[int] = []

    def append_waiting_count(self, waiting_count: int) -> None:
        self._waiting_count_list.append(waiting_count)

    def save_waiting_count_plot(self, save_path: str) -> None:
        Utils.save_plot(
            xdata=list(range(len(self._waiting_count_list))),
            ydata=self._waiting_count_list,
            title="Waiting for Data Count",
            xlabel="Steps",
            ylabel="Waiting Count",
            save_path=save_path
        )