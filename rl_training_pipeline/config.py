from enum import Enum

class ValidMode(Enum):
    TRAIN: str = '1'
    INFERENCE: str = '2'


class Config:

    # user cli
    VALID_USER_INPUT_MODE: list[str] = [ValidMode.TRAIN.value, ValidMode.INFERENCE.value]

    # PPO model manage
    LOAD_MODEL_PATH: str = "./rl_package/Model/PPO_2025-04-07.pt"
    SAVE_MODEL_PATH: str = "./rl_package/Model/PPO_2025-04-07.pt"
    
    SAVE_MODEL_FREQUENCY: int = 1024
    TRAINING_STEPS: int = 1024 * 20
    LOG_INTERVAL: int = 1
    
    LEARNING_RATE: float = 0.001
    N_STEPS: int = 1024
    BATCH_SIZE: int = 64
    N_EPOCHS: int = 10
    GAMMA: float = 0.995

    # stable baselines3 env
    ACTION_NVEC: int = 1
    TERMINATE_THRESHOLD: float = 4.0 # degree
    # WATIING_TIME_PER_STEP: float = 0.01 # second

    # env
    SERVO_STEP_ANGLE: float = 0.01 # degree

    # action
    MAX_JOINT_ANGLE: float = 3.0 # degree

    # reward
    STABLE_REWARD_WEIGHT: float = 0.5 # positive
    DELTA_FOOT_ANGLE_REWARD_WEIGHT: float = -42000.0 # negative
    FLIP_BONUS: float = 500.0 # positive

    # waiting data time monitor
    WAITING_TIME_STOP_THRESHOLD: int =  500000 # µs
    WAITING_DATA_TIME_PLOT_PATH: str = "./plots/waiting_data_time_plot.png"

    # Fps monitor
    FPS_LOG_INTERVAL: int = 32 # training steps
    FPS_PLOT_PATH: str = "./plots/fps_plot.png"

    LOW_FPS_THRESHOLD: float = 0.0
    MAX_LOW_FPS_STREAK: int = 4
    FPS_STOPPER_SLEEP_TIME: float = 10.0

    # reward monitor
    REWARD_PLOT_PATH: str = "./plots/reward_plot.png"
    AVERAGE_REWARD_PLOT_PATH: str = "./plots/average_reward_plot.png"

    # duration steps monitor
    DURATION_STEPS_PLOT_PATH: str = "./plots/duration_steps_plot.png"
    AVERATE_DURATION_STEPS_PLOT_PATH: str = "./plots/average_duration_steps_plot.png"

    # data dictionary keys
    THIGH_ANGLE_KEY: str = "thigh_angle"
    CALF_ANGLE_KEY: str = "calf_angle"
    FOOT_ANGLE_KEY: str = "foot_angle"

    # noise level
    OBS_NOISE_LEVEL: float = 0.0
    ACTION_NOISE_LEVEL: float = 0.0