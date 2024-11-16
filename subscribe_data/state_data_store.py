from std_msgs.msg import Float32MultiArray

class StateDataStore:
    
    def __init__(self) -> None:
        self._recieved_unity_data: dict = {}

    def store_data(self, key: str, value: Float32MultiArray) -> None:
        self._recieved_unity_data[key] = value

        return None