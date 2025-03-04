import copy
import sys

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
        unity_data: dict[str, float] = {}
        data_transformed: dict[str, float] = {}
        
        unity_data = self.unity_data_store.get_unity_data()
        data_decomposed = self._decomposition_data(unity_data)
        data_transformed = self._add_key(data_decomposed)
        self.pre_data_state = copy.deepcopy(data_decomposed)
        self.prepre_data_state = copy.deepcopy(self.pre_data_state)
        return data_transformed
        
    def _decomposition_data(self, unity_data: dict[str, float]) -> dict[str, float]:
        data_decomposed: dict[str, float] = {
            Config.TOP_ANGLE_KEY: unity_data[Config.TOP_ANGLE_KEY],
            Config.CALF_ANGLE_KEY: unity_data[Config.CALF_ANGLE_KEY],
        }
        sys.stderr.write(f"top angle: {data_decomposed[Config.TOP_ANGLE_KEY]:.3f}\n")
        return data_decomposed
    
    def _add_key(self, data_decomposed: dict[str, float]) -> dict[str, float]:
        
        data_added = self._add_pre_state(data_decomposed)
        data_added = self._add_prepre_state(data_added)
        
        return data_added
    
    def _add_pre_state(self, data_decomposed: dict[str, float]) -> dict[str, float]:
        data_decomposed["pre_"+Config.TOP_ANGLE_KEY] = self.pre_data_state.get(Config.TOP_ANGLE_KEY, 0)
        data_decomposed["pre_"+Config.CALF_ANGLE_KEY] = self.pre_data_state.get(Config.CALF_ANGLE_KEY, 0)
    
        return data_decomposed
    
    def _add_prepre_state(self, data_decomposed: dict[str, float]) -> dict[str, float]:
        data_decomposed["prepre_"+Config.TOP_ANGLE_KEY] = self.prepre_data_state.get("pre_"+Config.TOP_ANGLE_KEY, 0)
        data_decomposed["prepre_"+Config.CALF_ANGLE_KEY] = self.prepre_data_state.get("pre_"+Config.CALF_ANGLE_KEY, 0)
    
        return data_decomposed