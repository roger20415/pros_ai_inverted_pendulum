import copy
import time

from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from subscribe_data.waiting_data_monitor import WaitingDataMonitor

from config import Config

class UnityDataStore:
    
    def __init__(self) -> None:
        self._recieved_unity_data: dict[str, Float32] = {}
        
        self._if_data_ready_flags: dict[str, bool] = {
            Config.CALF_ANGLE_KEY: False,
            Config.TOP_ANGLE_KEY: False,
            Config.CALF_CENTER_OF_MASS_KEY: False,
            Config.TOP_CENTER_OF_MASS_KEY: False
        }
    
    def get_unity_data(self) -> dict:
        return copy.deepcopy(self._recieved_unity_data)

    def split_and_store_received_array(self, msg: Float32MultiArray) -> None:
        self._store_received_data(Config.TOP_ANGLE_KEY, msg.data[0])
        self._store_received_data(Config.CALF_ANGLE_KEY, msg.data[1])
        self._store_received_data(Config.TOP_CENTER_OF_MASS_KEY, msg.data[2])
        self._store_received_data(Config.CALF_CENTER_OF_MASS_KEY, msg.data[3])
        
    def wait_all_data_ready(self, waiting_data_monitor: WaitingDataMonitor, timeout: float = 0.1) -> None:
        start_time: float = time.time()
        last_log_time: float = start_time
        while not self._check_if_all_data_ready():
            if time.time() - last_log_time > timeout:
                print("Waiting for Unity data timed out.")
                last_log_time = time.time()
        waiting_data_monitor.append_waiting_time(int((time.time() - start_time) * 1e6)) # convert s to µs
        
    def turn_all_data_flag_to_unready(self) -> None:
        self._if_data_ready_flags = dict.fromkeys(self._if_data_ready_flags.keys(), False)
        
    def _check_if_all_data_ready(self) -> bool:
        if_all_data_ready: bool = False
        if all(self._if_data_ready_flags.values()):
            if_all_data_ready = True
        
        return if_all_data_ready

    def _store_received_data(self, key: str, data: float) -> None:
        self._recieved_unity_data[key] = data
        self._if_data_ready_flags[key] = True
        