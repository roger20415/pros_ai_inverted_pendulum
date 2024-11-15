from enum import Enum

class ValidMode(Enum):
    TRAIN: str = '1'
    INFERENCE: str = '2'


class Config:

    # user cli
    VALID_USER_INPUT_MODE: list[str] = [ValidMode.TRAIN.value, ValidMode.INFERENCE.value]




    # PPO training
    ACTION_NVEC: list[int] = [2]