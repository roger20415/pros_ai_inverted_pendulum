from enum import Enum

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
    LOAD_MODEL_PATH: str = "./Model/inverted_pendulum_PPO_2024-11-20.pt"
    SAVE_MODEL_PATH: str = "./Model/inverted_pendulum_PPO_2024-11-20.pt"
    
    SAVE_MODEL_FREQUENCY: int = 1024 * 128
    TRAINING_STEPS: int = 1024 * 128 * 128
    LOG_INTERVAL: int = 1
    
    LEARNING_RATE: float = 0.001
    N_STEPS: int = 1024
    BATCH_SIZE: int = 64
    N_EPOCHS: int = 10

    # PPO training
    ACTION_NVEC: list[int] = [9]
    
    
    # action
    JOINT_DELTA_UNIT: float = 1.0 # 
    MAX_JOINT_ANGLE: float = 120.0 # degrees