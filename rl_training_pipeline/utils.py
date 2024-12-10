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
    def save_plot(xdata, ydata, title, xlabel, ylabel, save_path, plot_type="line"):
        plt.figure(figsize=(10, 6))

        if plot_type == "line":
            plt.plot(xdata, ydata, label='Data', linestyle='-', marker='o')
        elif plot_type == "scatter":
            plt.scatter(xdata, ydata, label='Data', color='blue')

        plt.title(title)
        plt.xlabel(xlabel)
        plt.ylabel(ylabel)
        plt.legend()
        plt.grid(True)
        plt.savefig(save_path)
        plt.close()