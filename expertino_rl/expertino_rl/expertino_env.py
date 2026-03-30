from cx_rl_gym.cx_rl_gym import CXRLGym
from rclpy.node import Node
import rclpy

from expertino_rl_interfaces.action import StartRefbox, StopRefbox

from rclpy.action import ActionServer

import os
import subprocess
from ament_index_python.packages import get_package_share_directory




class ExpertinoEnv(CXRLGym):
    def __init__(self, node: Node, mode: str, number_robots: int):
        super().__init__(node, mode, number_robots)
        
        self.reward_in_episode = 0
        self.episode_number = 0
        
        self.expertino_dir = get_package_share_directory('expertino_rl')
        self.start_refbox_server = ActionServer(self.node, StartRefbox, 'start_refbox', self.start_refbox_callback)
        self.stop_refbox_server = ActionServer(self.node, StopRefbox, 'stop_refbox', self.stop_refbox_callback)
        

    def step(self, action):
        with open("cxrl-rcll-log-episode-reward.txt", 'a+') as f:
            f.write(f"{self.action_dict[action]} \n")
        state, reward, done, truncated, info = super().step(action)
        self.reward_in_episode += reward
        return state, reward, done, truncated, info
    
    def reset(self, seed: int = None, options: dict[str, any] = None):
        with open("cxrl-rcll-log-episode-reward.txt", 'a+') as f:
            f.write(f"{self.reward_in_episode} \n")
        self.node.get_logger().info(f"Episode {self.episode_number}.")
        self.episode_number += 1
        self.reward_in_episode = 0
        
        state, info = super().reset(seed=seed)
        
        return (state,info)
    """
    def generate_action_space(self):
        self.node.get_logger().info("Generating action space...")
        action_space =  ["spawn-and-transport#o1",
                         "transport#o1",
                         "base-transport#o1",
                         "spawn-and-transport#o2",
                         "transport#o2",
                         "base-transport#o2",
                         "spawn-and-transport#o3",
                         "transport#o3",
                         "base-transport#o3",
                         "spawn-and-transport#o4",
                         "transport#o4",
                         "base-transport#o4",
                         "spawn-and-transport#o5",
                         "transport#o5",
                         "base-transport#o5",
                         "spawn-and-transport#o6",
                         "transport#o6",
                         "base-transport#o6",
                         "spawn-and-transport#o7",
                         "transport#o7",
                         "base-transport#o7",
                         "spawn-and-transport#o8",
                         "transport#o8",
                         "base-transport#o8",
                         "spawn-and-transport#o9",
                         "transport#o9",
                         "base-transport#o9",
                         "spawn-and-transport#o10",
                         "transport#o10",
                         "base-transport#o10",
                         "pay-with-carrier#rs1#black-carrier",
                         "pay-with-carrier#rs2#black-carrier",
                         "pay-with-carrier#rs1#grey-carrier",
                         "pay-with-carrier#rs2#grey-carrier",
                         "carrier-to-input#black-carrier#cs1",
                         "carrier-to-input#black-carrier#cs2",
                         "carrier-to-input#grey-carrier#cs1",
                         "carrier-to-input#grey-carrier#cs2",
                         "transport-to-slide#black-carrier#rs1",
                         "transport-to-slide#black-carrier#rs2",
                         "transport-to-slide#grey-carrier#rs1",
                         "transport-to-slide#grey-carrier#rs2",
                         "pay-from-bs#rs1",
                         "pay-from-bs#rs2"
                        ]       
        return action_space
    """

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
