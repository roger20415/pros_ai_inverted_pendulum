import rclpy
from rclpy.node import Node
import gymnasium as gym

from config import ValidMode
from user_cli import UserCLI
from ros_node_manage import RosNodeManager
from rl_package.ppo_model_manage import PPOModelManager
from subscribe_data.data_manage import DataManager
from publish_action.action_manage import ActionManager

def main() -> None:
    rclpy.init()
    user_cli = UserCLI()
    ppo_model_manager = PPOModelManager()
    data_manager = DataManager()
    action_manager = ActionManager()
    all_ros_nodes: dict[str, Node] = {
        "data_subscriber": data_manager.get_data_subscriber_node(),
        "action_publisher": action_manager.get_action_publisher_node()
    }
    ros_node_manager = RosNodeManager(all_ros_nodes)
    
    try:
        user_cli.execute_user_cli()
        user_input_mode: str = user_cli.get_user_input_mode()

        ros_node_manager.start_multi_threaded_executor()
        env: gym.Env =  ppo_model_manager.register_gym_env(data_manager, action_manager)

        if user_input_mode == ValidMode.TRAIN.value:
            ppo_model_manager.train_model(env)
    
    except KeyboardInterrupt:
        print("Shutting down...")
        
    finally:
        ros_node_manager.shotdown_multi_threaded_executor()
        
    rclpy.shutdown()
    return None

if __name__ == "__main__":
    main()