from enum import Enum
from multiprocessing.pool import TERMINATE
from re import M

class ValidMode(Enum):
    TRAIN: str = '1'
    INFERENCE: str = '2'


class Config:

    # unity config
    BASE_LINK_MASS: float = 141.3327
    CALF_MASS: float = 29.71538

    # user cli
    VALID_USER_INPUT_MODE: list[str] = [ValidMode.TRAIN.value, ValidMode.INFERENCE.value]

    # PPO model manage
    LOAD_MODEL_PATH: str = "./rl_package/Model/inverted_pendulum_PPO_2024-11-20.pt"
    SAVE_MODEL_PATH: str = "./rl_package/Model/inverted_pendulum_PPO_2024-11-20.pt"
    
    SAVE_MODEL_FREQUENCY: int = 1024
    TRAINING_STEPS: int = 1024 * 100
    LOG_INTERVAL: int = 1
    
    LEARNING_RATE: float = 0.001
    N_STEPS: int = 1024
    BATCH_SIZE: int = 64
    N_EPOCHS: int = 10
    GAMMA: float = 0.995

    # stable baselines3 env
    ACTION_NVEC: list[int] = [11]
    NO_MOVE_ACTION: int = 0
    POSITIVE_ACTIONS: list[int] = [1, 2, 3, 4, 5]
    NEGATIVE_ACTIONS: list[int] = [6, 7, 8, 9, 10]
    TERMINATE_THRESHOLD: float = 15.0 # degree
    WATIING_TIME_PER_STEP: float = 0.0025 # second

    # action
    JOINT_DELTA_UNIT: float = 4 # degree
    MAX_JOINT_ANGLE: float = 120.0 # degree

    # reward
    SWING_ALIGNMENT_REWARD: float = 5.0 # reward points
    CENTER_REWARD_WEIGHT: float = -2000.0
    STABILITY_BONUS_THRESHOLD: float = 1.5 # degree
    STABILITY_BONUS: float = 4.0 # reward points
    TILT_PENALTY_THRESHOLD: float = 4.5 # degree
    TILT_PENALTY: float = -10.0 # reward points

    STEP_DURATION_GROWTH_RATE: float = 0.42
    STEP_COUNT_THRESHOLD: int = 9

    # waiting data time monitor
    WAITING_TIME_STOP_THRESHOLD: int =  500000 # µs
    WAITING_DATA_TIME_PLOT_PATH: str = "./waiting_data_time_plot.png"

    # Fps monitor
    FPS_LOG_INTERVAL: int = 32 # training steps
    FPS_PLOT_PATH: str = "./rl_package/fps_plot.png"

    LOW_FPS_THRESHOLD: float = 0.0
    MAX_LOW_FPS_STREAK: int = 4
    FPS_STOPPER_SLEEP_TIME: float = 10.0

    # com monitor 
    COM_PLOT_PATH: str = "./subscribe_data/com_plot.png"

    # duration steps monitor
    DURATION_STEPS_PLOT_PATH: str = "./rl_package/duration_steps_plot.png"
    AVERATE_DURATION_STEPS_PLOT_PATH: str = "./rl_package/average_duration_steps_plot.png"