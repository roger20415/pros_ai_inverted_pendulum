from utils import Utils

class RewardMonitor:
    def __init__(self) -> None:
        self._reward_list: list[float] = []
        self._average_reward_list: list[float] = []

    def add_reward(self, reward: float) -> None:
        self._reward_list.append(reward)
        self._update_average_reward()

    def _update_average_reward(self) -> None:
        sample_epochs = self._reward_list[-50:]
        avg = sum(sample_epochs) / len(sample_epochs)
        self._average_reward_list.append(avg)

    def save_avg_reward_plot(self, save_path: str) -> None:
        Utils.save_plot(
            xdata=list(range(len(self._average_reward_list))),
            ydata=self._average_reward_list,
            title="Average Reward",
            xlabel="Epoch",
            ylabel="Average Reward",
            save_path=save_path,
            plot_type="line"
        )

    def save_reward_plot(self, save_path: str) -> None:
        Utils.save_plot(
            xdata=list(range(len(self._reward_list))),
            ydata=self._reward_list,
            title="Reward",
            xlabel="Epoch",
            ylabel="Reward",
            save_path=save_path,
            plot_type="scatter",
        )
