from enum import Enum

class ValidMode(Enum):
    TRAIN: str = '1'
    INFERENCE: str = '2'


class Config:

    # user cli
    VALID_USER_INPUT_MODE: list[str] = [ValidMode.TRAIN.value, ValidMode.INFERENCE.value]




    # PPO training
    ACTION_NVEC: list[int] = [2]
    
    
    # action
    JOINT_DELTA_UNIT: float = 0.5 # 
    MAX_JOINT_ANGLE: float = 120.0 # degrees