import os
import logging

from stable_baselines3.common.callbacks import BaseCallback

class SaveModelCallback(BaseCallback):
    def __init__(self, save_path: str, save_freq: int, verbose: int = 0) -> None:
        super(SaveModelCallback, self).__init__(verbose)
        
        logging.basicConfig(level = logging.INFO)
        
        self._save_path: str = save_path
        self._save_freq = save_freq
        self._last_save_step = 0
        
        return None
    
    def _on_step(self) -> bool:

        if self.n_calls - self._last_save_step >= self._save_freq:
            
            save_dir = os.path.dirname(self._save_path) 
            if not os.path.exists(save_dir):
                os.makedirs(save_dir)
                self.logger.info(f"Created new directory: {save_dir}")
            
            self.logger.info(f"Run {self.n_calls} steps...")
            self.model.save(self._save_path)
            self.logger.info(f"Model saved to {self._save_path}")
            self._last_save_step = self.n_calls

        # _on_step() should return True to continue training
        return True   