import rclpy
from rclpy.node import Node
import gymnasium as gym

from config import ValidMode
from config import Config
from user_cli import UserCLI
from ros_node_manage import RosNodeManager
from rl_package.ppo_model_manage import PPOModelManager
from subscribe_data.data_manage import DataManager
from publish_action.action_manage import ActionManager
from unity_state.unity_state_communication import UnityStateManagerNode
from rl_package.callbacks.callback_manager import CallbackManager
from stable_baselines3.common.callbacks import BaseCallback

def main() -> None:
    rclpy.init()
    user_cli = UserCLI()
    ppo_model_manager = PPOModelManager()
    data_manager = DataManager()
    action_manager = ActionManager()
    unity_state_manager = UnityStateManagerNode()
    callback_manager = CallbackManager()

    all_ros_nodes: dict[str, Node] = {
        "data_subscriber": data_manager.get_data_subscriber_node(),
        "action_publisher": action_manager.get_action_publisher_node(),
        "unity_state_manager": unity_state_manager
    }
    callbacks: list[BaseCallback] = callback_manager.get_callbacks()
    ros_node_manager = RosNodeManager(all_ros_nodes)
    
    try:
        user_cli.execute_user_cli()
        user_input_mode: str = user_cli.get_user_input_mode()

        ros_node_manager.start_multi_threaded_executor()
        env: gym.Env =  ppo_model_manager.register_gym_env(data_manager, action_manager, unity_state_manager)

        if user_input_mode == ValidMode.TRAIN.value:
            ppo_model_manager.train_model(env, callbacks)
    
    except KeyboardInterrupt:
        print("Shutting down...")
        
    finally:
        ros_node_manager.shotdown_multi_threaded_executor()

    data_manager.waiting_data_monitor.save_waiting_time_plot(Config.WAITING_DATA_TIME_PLOT_PATH)
        
    rclpy.shutdown()
    

if __name__ == "__main__":
    main()