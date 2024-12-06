from utils import Utils

# COM: Center of Mass
class DurationStepMonitor:
    def __init__(self) -> None:
        self._duration_steps: int = 0
        self._duration_steps_list: list[int] = []

    def add_duration_steps(self) -> None:
        self._duration_steps += 1

    def append_duration_steps_to_list(self) -> None:
        self._duration_steps_list.append(self._duration_steps)
        self._reset_duration_steps()

    def _reset_duration_steps(self) -> None:
        self._duration_steps = 0

    def save_duration_steps_plot(self, save_path: str) -> None:
        Utils.save_plot(
            xdata=list(range(len(self._duration_steps_list))),
            ydata=self._duration_steps_list,
            title="Duration steps",
            xlabel="Epoch",
            ylabel="Duration steps",
            save_path=save_path
        )