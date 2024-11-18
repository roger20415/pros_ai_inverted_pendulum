import math
import sys

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
    
    @staticmethod
    def radians_degrees_transfer(data: list[float] , mode: str) -> list[float]:
        """
        Transfer a list from degrees to radians or radians to degrees.

        Parameters
        ----------
            data: list[float]
                The data want to be transfered.
            mode: str
                degrees2radians or radians2degrees.
        
        Returns
        ----------
            data: list[float]
                The data transfered.

        Raises
        ----------
            Value Error
                If the mode(str) is not "degrees2radians" or "radians2degrees" will cause an error.
        """
        
        if (mode == "degrees2radians"):
            data = [math.radians(deg) for deg in data]
        elif (mode == "radians2degrees"):
            data = [math.degrees(deg) for deg in data]
        else:
            print("Error: The mode should be degrees2radians or radians2degrees...")
            sys.exit()

        return data
    