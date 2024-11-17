from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rl_training_pipeline.subscribe_data.data_manage import DataManager
from rl_training_pipeline.publish_action.action_manage import ActionManager

class RosNodeManager:
    def __init__(self):
        self.data_manager = DataManager()
        self.action_manager = ActionManager()
        self.multi_threaded_executor = MultiThreadedExecutor()
        
        self.data_subscriber_node: Node = self.data_manager.get_data_subscriber_node()
        self.action_publisher_node: Node = self.action_manager.get_action_publisher_node()
        
    def start_multi_threaded_executor(self) -> None:

        all_ros_nodes: dict[str, Node] = self.get_all_ros_nodes()
        for ros_node in all_ros_nodes.values():
            self.multi_threaded_executor.add_node(ros_node)
            
        self.multi_threaded_executor.spin()
        
        return None
    
    def shotdown_multi_threaded_executor(self) -> None:
        self.multi_threaded_executor.shutdown()
        
        all_ros_nodes: dict[str, Node] = self.get_all_ros_nodes()
        for ros_node in all_ros_nodes.values():
            ros_node.destroy_node()
        
        return None
                
    def get_all_ros_nodes(self) -> dict[str, Node]:
        
        all_ros_nodes: dict[str, Node] = {
            "data_subscriber_node": self.data_subscriber_node,
            "action_publisher_node": self.action_publisher_node
        }
        return all_ros_nodes