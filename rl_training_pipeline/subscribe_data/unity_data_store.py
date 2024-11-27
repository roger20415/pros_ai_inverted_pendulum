import copy
import time

from std_msgs.msg import Float32MultiArray
from subscribe_data.waiting_data_monitor import WaitingDataMonitor

class UnityDataStore:
    
    def __init__(self) -> None:
        self._recieved_unity_data: dict[str, Float32MultiArray] = {}
        
        self._if_data_ready_flags: dict[str, bool] = {
            "calf_angle": False,
            "foundation_angle": False,
            "baselink_center_of_mass": False,
            "calf_center_of_mass": False
        }

        self.waiting_data_monitor = WaitingDataMonitor()
    
    def get_unity_data(self) -> dict:
        return copy.deepcopy(self._recieved_unity_data)

    def store_received_data(self, key: str, msg: Float32MultiArray) -> None:

        self._recieved_unity_data[key] = msg.data
        self._turn_data_flag_to_ready(key)

    def wait_all_data_ready(self, timeout: float = 0.1) -> None:
        start_time: float = time.time()
        last_log_time: float = start_time
        while not self._check_if_all_data_ready():
            if time.time() - last_log_time > timeout:
                print("Waiting for Unity data timed out.")
                last_log_time = time.time()
            time.sleep(0.005)
        self.waiting_data_monitor.append_waiting_time(int((time.time() - start_time) * 1e6)) # convert s to µs
        
        
    def turn_all_data_flag_to_unready(self) -> None:
        self._if_data_ready_flags = dict.fromkeys(self._if_data_ready_flags.keys(), False)
        
    def _check_if_all_data_ready(self) -> bool:
        if_all_data_ready: bool = False
        if all(self._if_data_ready_flags.values()):
            if_all_data_ready = True
        
        return if_all_data_ready
    
    def _turn_data_flag_to_ready(self, key: str) -> None:
        
        self._if_data_ready_flags[key] = True
        