from utils import Utils

# COM: Center of Mass
class ComMonitor:
    def __init__(self) -> None:
        self._com_list: list[float] = []

    def append_com(self, com: float) -> None:
        self._com_list.append(com)

    def save_com_plot(self, save_path: str) -> None:
        Utils.save_plot(
            xdata=list(range(len(self._com_list))),
            ydata=self._com_list,
            title="Center of mass",
            xlabel="Steps",
            ylabel="Center of mass",
            save_path=save_path
        )