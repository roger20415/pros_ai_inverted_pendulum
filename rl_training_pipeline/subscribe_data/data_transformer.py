from subscribe_data.unity_data_store import UnityDataStore
from std_msgs.msg import Float32MultiArray

class DataTransformer():
    def __init__(self, unity_data_store: UnityDataStore) -> None:
        
        self.unity_data_store = unity_data_store
        return None
    
    def transform_untiy_data_to_state(self) -> dict[str, float]:
        
        unity_data: dict[str, Float32MultiArray] = {}
        data_transformed = dict[str, float] = {}
        
        unity_data = self.unity_data_store.get_unity_data()
        data_transformed = self._decomposition_data(unity_data)
        
        return data_transformed
        
    def _decomposition_data(self, unity_data: dict[str, Float32MultiArray]) -> dict[str, float]:
        
        data_transformed: dict[str, float] = {
           "calf_angle": unity_data["calf_angle"][0] 
        }
        
        return data_transformed
        