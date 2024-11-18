import numpy as np

class Utils:
    
    @staticmethod
    def flatten_dict_to_array(dictionay: dict) -> np.ndarray:
        
        flat_list: list = []
        for value in dictionay.values():
            if isinstance(value, list):
                flat_list.extend(value)
            else:
                flat_list.append(value)
        flat_array = np.array(flat_list, dtype = np.float32)
        
        return flat_array