import numpy as np
import matplotlib.pyplot as plt

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
    def save_plot(xdata: list[float | int], ydata: list[float | int], 
                title: str, xlabel: str, ylabel: str, save_path: str) -> None:
        
        plt.clf()
        plt.plot(xdata, ydata)
        plt.title(title)
        plt.xlabel(xlabel)
        plt.ylabel(ylabel)
        plt.savefig(save_path)
        print(f"Plot saved as {save_path}")