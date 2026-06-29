from cx_rl_gym.cx_rl_gym import CXRLGym
from rclpy.node import Node
import rclpy

from expertino_rl_interfaces.action import StartRefbox, StopRefbox

from rclpy.action import ActionServer

import os
import subprocess
from ament_index_python.packages import get_package_share_directory




class ExpertinoEnv(CXRLGym):
    def __init__(self, node: Node, mode: str):
        super().__init__(node, mode)
        
        self.reward_in_episode = 0
        self.episode_number = 0
        
        self.expertino_dir = get_package_share_directory('expertino_rl')
        self.start_refbox_server = ActionServer(self.node, StartRefbox, 'cx_rl_node/start_refbox', self.start_refbox_callback)
        self.stop_refbox_server = ActionServer(self.node, StopRefbox, 'cx_rl_node/stop_refbox', self.stop_refbox_callback)

        self.log_file = os.path.join(self.node.log_dir, "rewards.txt")
        

    def step(self, action):
        state, reward, done, truncated, info = super().step(action)
        self.reward_in_episode += reward
        return state, reward, done, truncated, info
    
    def reset(self, seed: int = None, options: dict[str, any] = None):
        with open(self.log_file, 'a+') as f:
            f.write(f"{self.reward_in_episode} \n")
        self.node.get_logger().info(f"Episode {self.episode_number}.")
        self.episode_number += 1
        self.reward_in_episode = 0
        
        state, info = super().reset(seed=seed)
        
        return (state,info)

    def render(self):
        pass
    
    def start_refbox_callback(self, goal_handle):
        self.node.get_logger().info("Starting refbox...")

        start_path = os.path.join(self.expertino_dir + "/scripts/start_refbox.sh")
        subprocess.call(['sh', start_path])
        
        goal_handle.succeed()
        result = StartRefbox.Result()
        result.success = True
        return result
        
    def stop_refbox_callback(self, goal_handle):
        self.node.get_logger().info("stopping refbox...")
        stop_path = os.path.join(self.expertino_dir + "/scripts/stop_refbox.sh")
        subprocess.call(['sh', stop_path])
        
        goal_handle.succeed()
        result = StopRefbox.Result()
        result.success = True
        return result
