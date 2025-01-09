from enum import Enum

class ValidMode(Enum):
    TRAIN: str = '1'
    INFERENCE: str = '2'


class Config:

    # unity config
    TOP_MASS: float = 0.08000553
    CALF_MASS: float = 0.07169855

    # user cli
    VALID_USER_INPUT_MODE: list[str] = [ValidMode.TRAIN.value, ValidMode.INFERENCE.value]

    # PPO model manage
    LOAD_MODEL_PATH: str = "./rl_package/Model/inverted_pendulum_PPO_2024-11-20.pt"
    SAVE_MODEL_PATH: str = "./rl_package/Model/inverted_pendulum_PPO_2024-11-20.pt"
    
    SAVE_MODEL_FREQUENCY: int = 1024
    TRAINING_STEPS: int = 1024 * 10
    LOG_INTERVAL: int = 1
    
    LEARNING_RATE: float = 0.001
    N_STEPS: int = 1024
    BATCH_SIZE: int = 64
    N_EPOCHS: int = 10
    GAMMA: float = 0.995

    # stable baselines3 env
    ACTION_NVEC: int = 1
    NO_MOVE_ACTION: int = 0
    POSITIVE_ACTIONS: list[int] = [1, 2, 3, 4, 5]
    NEGATIVE_ACTIONS: list[int] = [6, 7, 8, 9, 10]
    TERMINATE_THRESHOLD: float = 20.0 # degree
    WATIING_TIME_PER_STEP: float = 0.035 # second

    # action
    JOINT_DELTA_UNIT: float = 3.0 # degree
    MAX_JOINT_ANGLE: float = 3.0 # degree

    # reward
    STABLE_THRESHOLD: float = 0.5 # degree
    STABLE_REWARD_WEIGHT: float = 1.0 # positive
    CENTER_OF_MASS_REWARD_WEIGHT: float = -200000.0 # negative

    # waiting data time monitor
    WAITING_TIME_STOP_THRESHOLD: int =  500000 # µs
    WAITING_DATA_TIME_PLOT_PATH: str = "./plots/waiting_data_time_plot.png"

    # Fps monitor
    FPS_LOG_INTERVAL: int = 32 # training steps
    FPS_PLOT_PATH: str = "./plots/fps_plot.png"

    LOW_FPS_THRESHOLD: float = 0.0
    MAX_LOW_FPS_STREAK: int = 4
    FPS_STOPPER_SLEEP_TIME: float = 10.0

    # com monitor 
    COM_PLOT_PATH: str = "./plots/com_plot.png"

    # reward monitor
    REWARD_PLOT_PATH: str = "./plots/reward_plot.png"
    AVERAGE_REWARD_PLOT_PATH: str = "./plots/average_reward_plot.png"

    # duration steps monitor
    DURATION_STEPS_PLOT_PATH: str = "./plots/duration_steps_plot.png"
    AVERATE_DURATION_STEPS_PLOT_PATH: str = "./plots/average_duration_steps_plot.png"

    CALF_ANGLE_KEY: str = "calf_angle"
    TOP_ANGLE_KEY: str = "top_angle"
    CALF_CENTER_OF_MASS_KEY: str = "calf_center_of_mass"
    TOP_CENTER_OF_MASS_KEY: str = "top_center_of_mass"
    CENTER_OF_MASS_KEY: str = "center_of_mass"