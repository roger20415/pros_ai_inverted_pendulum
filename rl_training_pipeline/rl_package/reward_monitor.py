from utils import Utils

class RewardMonitor:
    def __init__(self) -> None:
        self._reward_list: list[float] = []
        self._average_reward_list: list[float] = []

        self._stable_reward_list: list[float] = []
        self._foundation_angle_reward_list: list[float] = []
        self._flip_bonus_list: list[float] = []

    def add_reward(self, reward: float) -> None:
        self._reward_list.append(reward)
        self._update_average_reward()

    def add_stable_reward(self, reward: float) -> None:
        self._stable_reward_list.append(reward)

    def add_foundation_angle_reward(self, reward: float) -> None:
        self._foundation_angle_reward_list.append(reward)

    def add_flip_bonus(self, bonus: float) -> None:
        self._flip_bonus_list.append(bonus)

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

    def save_foundation_angle_reward_plot(self, save_path: str) -> None:
        Utils.save_plot(
            xdata=list(range(len(self._foundation_angle_reward_list))),
            ydata=self._foundation_angle_reward_list,
            title="Foundation Angle Reward",
            xlabel="Epoch",
            ylabel="Foundation Angle Reward",
            save_path=save_path,
            plot_type="line"
        )

    def save_flip_bonus_plot(self, save_path: str) -> None:
        Utils.save_plot(
            xdata=list(range(len(self._flip_bonus_list))),
            ydata=self._flip_bonus_list,
            title="Flip Bonus",
            xlabel="Epoch",
            ylabel="Flip Bonus",
            save_path=save_path,
            plot_type="line"
        )

    def save_combined_reward_plot(self, save_path: str) -> None:
        def moving_average(data: list[float], window: int = 200) -> list[float]:
            avg_list = []
            for i in range(len(data)):
                window_data = data[max(0, i - window + 1): i + 1]
                avg = sum(window_data) / len(window_data)
                avg_list.append(avg)
            return avg_list

        length = min(
            len(self._stable_reward_list),
            len(self._foundation_angle_reward_list),
            len(self._flip_bonus_list),
        )
        xdata = list(range(length))

        y_dict = {
            "Stable Reward": moving_average(self._stable_reward_list[:length]),
            "Delta Foundation Angle Reward": moving_average(self._foundation_angle_reward_list[:length]),
            "Flip Bonus": moving_average(self._flip_bonus_list[:length]),
        }

        Utils.save_multi_plot(
            xdata=xdata,
            ydata_dict=y_dict,
            title="Combined Reward Components (Smoothed)",
            xlabel="Epoch",
            ylabel="Reward Value",
            save_path=save_path,
        )

