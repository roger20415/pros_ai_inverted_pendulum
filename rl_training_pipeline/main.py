from user_cli import UserCLI

class MainProcessor:
    def __init__(self):
        self.user_cli = UserCLI()

    def _register_env():
        pass
    
    def _train_model():
        pass

if __name__ == "__main__":
    main_processor = MainProcessor()
    
    main_processor.user_cli.execute_user_cli()
    user_input_mode = main_processor.user_cli.get_user_input_mode()

    print("of", user_input_mode)