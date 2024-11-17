import rclpy

import gymnasium as gym

from rclpy.node import Node
from config import ValidMode
from user_cli import UserCLI
from ros_node_manage import RosNodeManager
from rl_training_pipeline.stable_baselines3.ppo_model_manage import PPOModelManager

def main() -> None:
    rclpy.init()
    ros_node_manager = RosNodeManager()
    user_cli = UserCLI()
    ppo_model_manager = PPOModelManager()
    all_ros_nodes: dict[str, Node] = ros_node_manager.get_all_ros_nodes()
    
    try:
        ros_node_manager.start_multi_threaded_executor()
        user_cli.execute_user_cli()
        user_input_mode: str = user_cli.get_user_input_mode()

        env: gym.Env =  ppo_model_manager.register_gym_env(all_ros_nodes)

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