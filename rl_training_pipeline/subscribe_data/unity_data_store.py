import copy
import time
import threading

from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from subscribe_data.waiting_data_monitor import WaitingDataMonitor

from config import Config

class UnityDataStore:
    
    def __init__(self) -> None:
        self._received_unity_data: dict[str, Float32] = {}
        
        self._if_data_ready_flags: dict[str, bool] = {
            Config.CALF_ANGLE_KEY: False,
            Config.TOP_ANGLE_KEY: False,
            Config.CALF_CENTER_OF_MASS_KEY: False,
            Config.TOP_CENTER_OF_MASS_KEY: False
        }

        self._data_ready_event = threading.Event()
    
    def get_unity_data(self) -> dict[str, Float32]:
        return copy.deepcopy(self._received_unity_data)

    def split_and_store_received_array(self, msg: Float32MultiArray) -> None:
        self._store_received_data(Config.TOP_ANGLE_KEY, msg.data[0])
        self._store_received_data(Config.CALF_ANGLE_KEY, msg.data[1])
        self._store_received_data(Config.TOP_CENTER_OF_MASS_KEY, msg.data[2])
        self._store_received_data(Config.CALF_CENTER_OF_MASS_KEY, msg.data[3])

        if self._check_if_all_data_ready():
            self._data_ready_event.set()
        
    def wait_all_data_ready(self, waiting_data_monitor: WaitingDataMonitor, timeout: float = 0.1) -> None:
        max_retries: int = 10
        retries: int = 0

        while retries < max_retries:
            try:
                data_ready = self._data_ready_event.wait(timeout)
                if not data_ready:
                    raise TimeoutError(f"Retry {retries+1}/{max_retries}: Unity data did not arrive in time!")
                break

            except TimeoutError as e:
                print(f"\033[93mWarning: {e}\033[0m")
                retries += 1

        if retries == max_retries:
            print("\033[91mError: Failed to receive Unity data.\033[0m")

        
    def turn_all_data_flag_to_unready(self) -> None:
        self._if_data_ready_flags = dict.fromkeys(self._if_data_ready_flags.keys(), False)
        self._data_ready_event.clear()
        
    def _check_if_all_data_ready(self) -> bool:
        if_all_data_ready: bool = False
        if all(self._if_data_ready_flags.values()):
            if_all_data_ready = True
        
        return if_all_data_ready

    def _store_received_data(self, key: str, data: float) -> None:
        self._received_unity_data[key] = data
        self._if_data_ready_flags[key] = True
        