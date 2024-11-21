from std_msgs.msg import Float32MultiArray

from subscribe_data.unity_data_store import UnityDataStore
from config import Config

class DataTransformer():
    def __init__(self, unity_data_store: UnityDataStore) -> None:
        
        self.unity_data_store = unity_data_store
        
    
    def transform_untiy_data_to_state(self) -> dict[str, float]:
        
        unity_data: dict[str, Float32MultiArray] = {}
        data_transformed: dict[str, float] = {}
        
        unity_data = self.unity_data_store.get_unity_data()
        data_transformed = self._decomposition_data(unity_data)
        
        return data_transformed
        
    def _decomposition_data(self, unity_data: dict[str, Float32MultiArray]) -> dict[str, float]:
        
        data_transformed: dict[str, float] = {
           "calf_angle": unity_data["calf_angle"][0],
           "foundation_angle": unity_data["foundation_angle"][0],
           "center_of_mass": self._calculate_center_of_mass(unity_data)
        }

        return data_transformed
    
    def _calculate_center_of_mass(self, unity_data: dict[str, Float32MultiArray]) -> float:
        
        total_mass: float = Config.BASE_LINK_MASS + Config.CALF_MASS
        weighted_mass: float = (unity_data["baselink_center_of_mass"][0] * Config.BASE_LINK_MASS + 
                                unity_data["calf_center_of_mass"][0] * Config.CALF_MASS)
        if total_mass is 0:
            raise ValueError("Total mass is zero. Cannot calculate center of mass.")
        center_of_mass: float = weighted_mass/total_mass

        return center_of_mass
        