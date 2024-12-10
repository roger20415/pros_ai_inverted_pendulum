import copy

from std_msgs.msg import Float32MultiArray

from subscribe_data.unity_data_store import UnityDataStore
from config import Config

class DataTransformer():
    def __init__(self, unity_data_store: UnityDataStore) -> None:
        
        self.unity_data_store = unity_data_store
        self.pre_data_state: dict[str, float] = {}
        self.prepre_data_state: dict[str, float] = {}
    
    def reset(self) -> None:
        
        self.pre_data_state = {}
        self.prepre_data_state = {}
        
    def transform_untiy_data_to_state(self) -> dict[str, float]:
        
        unity_data: dict[str, Float32MultiArray] = {}
        data_transformed: dict[str, float] = {}
        
        unity_data = self.unity_data_store.get_unity_data()
        data_decomposed = self._decomposition_data(unity_data)
        data_transformed = self._add_key(data_decomposed)

        self.pre_data_state = copy.deepcopy(data_decomposed)
        self.prepre_data_state = copy.deepcopy(self.pre_data_state)
        return data_transformed
        
    def _decomposition_data(self, unity_data: dict[str, Float32MultiArray]) -> dict[str, float]:
        
        data_decomposed: dict[str, float] = {
           "calf_angle": unity_data["calf_angle"][0],
           "foundation_angle": unity_data["foundation_angle"][0],
           "center_of_mass": self._calculate_center_of_mass(unity_data)
        }

        return data_decomposed
    
    def _calculate_center_of_mass(self, unity_data: dict[str, Float32MultiArray]) -> float:
        
        total_mass: float = Config.BASE_LINK_MASS + Config.CALF_MASS
        weighted_mass: float = (unity_data["baselink_center_of_mass"][0] * Config.BASE_LINK_MASS + 
                                unity_data["calf_center_of_mass"][0] * Config.CALF_MASS)
        if total_mass == 0:
            raise ValueError("Total mass is zero. Cannot calculate center of mass.")
        center_of_mass: float = weighted_mass/total_mass

        return center_of_mass
    
    def _add_key(self, data_decomposed: dict[str, float]) -> dict[str, float]:
        
        data_added = self._add_pre_state(data_decomposed)
        data_added = self._add_prepre_state(data_added)
        
        return data_added
    
    def _add_pre_state(self, data_decomposed: dict[str, float]) -> dict[str, float]:
        
        data_decomposed["pre_calf_angle"] = self.pre_data_state.get("calf_angle", 0)
        data_decomposed["pre_foundation_angle"] = self.pre_data_state.get("foundation_angle", 0)
        data_decomposed["pre_center_of_mass"] = self.pre_data_state.get("center_of_mass", 0)
    
        return data_decomposed
    
    def _add_prepre_state(self, data_decomposed: dict[str, float]) -> dict[str, float]:
        
        data_decomposed["prepre_calf_angle"] = self.prepre_data_state.get("pre_calf_angle", 0)
        data_decomposed["prepre_foundation_angle"] = self.prepre_data_state.get("pre_foundation_angle", 0)
        data_decomposed["prepre_center_of_mass"] = self.prepre_data_state.get("pre_center_of_mass", 0)
    
        return data_decomposed