from config import Config

class UserCLI:

    def __init__(self) -> None:
        self._user_input_mode: str = '0'

    def execute_user_cli(self) -> None:
        self._print_options()
        self._receive_user_input()
        self._check_user_input()
        print(f"Selected mode: {self._user_input_mode}")

    def get_user_input_mode(self) -> str:
        return self._user_input_mode
    
    def _print_options(self) -> None:
        print("Modes:")
        print("1. Train")
        print("2. Inference")
    
    def _receive_user_input(self) -> None:
        self._user_input_mode = input("Enter mode: ")
    
    def _check_user_input(self) -> None:
        if self._user_input_mode not in Config.VALID_USER_INPUT_MODE:
            raise ValueError(f"Invalid mode: {self._user_input_mode}. Please select from {Config.VALID_USER_INPUT_MODE}.")