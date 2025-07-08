from cxrl_gym.cxrl_gym import CXRLGym
from rclpy.node import Node
import rclpy


class BlocksworldEnv(CXRLGym):
    def __init__(self, node: Node, mode: str, number_robots: int):
        self.reward_in_episode = 0
        self.episode_number = 0
        super().__init__(node, mode, number_robots)

    def step(self, action):
        with open("cxrl-bw-log-episode-reward.txt", 'a+') as f:
            f.write(f"{self.action_dict[action]} \n")
        state, reward, done, truncated, info = super().step(action)
        self.reward_in_episode += reward
        return state, reward, done, truncated, info
    
    def reset(self, seed: int = None, options: dict[str, any] = None):
        with open("cxrl-bw-log-episode-reward.txt", 'a+') as f:
            f.write(f"{self.reward_in_episode} \n")
        self.node.get_logger().info(f"Episode {self.episode_number}.")
        self.episode_number += 1
        self.reward_in_episode = 0
        return super().reset(seed=seed)
    
    def generate_action_space(self):
        self.node.get_logger().info("Generating action space...")
        action_space =  ["stack#robot1#block1#block2",
                         "stack#robot1#block1#block3",
                         "stack#robot1#block1#block4",
                         "stack#robot1#block2#block1",
                         "stack#robot1#block2#block3",
                         "stack#robot1#block2#block4",
                         "stack#robot1#block3#block1",
                         "stack#robot1#block3#block2",
                         "stack#robot1#block3#block4",
                         "stack#robot1#block4#block1",
                         "stack#robot1#block4#block2",
                         "stack#robot1#block4#block3",
                         "pickup#robot1#block1",
                         "pickup#robot1#block2",
                         "pickup#robot1#block3",
                         "pickup#robot1#block4"
                        ]       
        return action_space

    def render(self):
        pass
