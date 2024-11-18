from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

class RosNodeManager:
    def __init__(self, all_ros_nodes: dict[str, Node]) -> None:
        self.multi_threaded_executor = MultiThreadedExecutor()
        self._all_ros_nodes: dict[str, Node] = all_ros_nodes

        return None
        
    def start_multi_threaded_executor(self) -> None:
        
        for ros_node in self._all_ros_nodes.values():
            self.multi_threaded_executor.add_node(ros_node)
          
        self.multi_threaded_executor.spin()
        return None
    
    def shotdown_multi_threaded_executor(self) -> None:
        self.multi_threaded_executor.shutdown()
        
        for ros_node in self._all_ros_nodes.values():
            ros_node.destroy_node()
        
        return None