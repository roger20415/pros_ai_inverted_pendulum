from enum import Enum
from multiprocessing.pool import TERMINATE
from re import M

class ValidMode(Enum):
    TRAIN: str = '1'
    INFERENCE: str = '2'


class Config:

    # unity config
    BASE_LINK_MASS: float = 47.885
    CALF_MASS: float = 29.71538

    # user cli
    VALID_USER_INPUT_MODE: list[str] = [ValidMode.TRAIN.value, ValidMode.INFERENCE.value]

    # PPO model manage
    LOAD_MODEL_PATH: str = "./rl_package/Model/inverted_pendulum_PPO_2024-11-20.pt"
    SAVE_MODEL_PATH: str = "./rl_package/Model/inverted_pendulum_PPO_2024-11-20.pt"
    
    SAVE_MODEL_FREQUENCY: int = 1024
    TRAINING_STEPS: int = 1024 * 2
    LOG_INTERVAL: int = 1
    
    LEARNING_RATE: float = 0.001
    N_STEPS: int = 1024
    BATCH_SIZE: int = 64
    N_EPOCHS: int = 10

    # stable baselines3 env
    ACTION_NVEC: list[int] = [11]
    TERMINATE_THRESHOLD: float = 6.0 # degree

    # action
    JOINT_DELTA_UNIT: float = 1.8 # degree
    MAX_JOINT_ANGLE: float = 120.0 # degree

    # reward
    CENTER_REWARD_WEIGHT: float = -5.0
    STABILITY_BONUS_THRESHOLD: float = 1.0 # degree
    STABILITY_BONUS: float = 4.0 # reward points
    TILT_PENALTY_THRESHOLD: float = 3.0 # degree
    TILT_PENALTY: float = -10.0 # reward pointsS


    WAITING_DATA_TIME_PLOT_PATH: str = "./waiting_data_time_plot.png"

    # Fps monitor
    FPS_LOG_INTERVAL: int = 32 # training steps
    FPS_PLOT_PATH: str = "./rl_package/fps_plot.png"

    LOW_FPS_THRESHOLD: float = 0.0
    MAX_LOW_FPS_STREAK: int = 4
    FPS_STOPPER_SLEEP_TIME: float = 10.0