'''
Example code on how to open a custom model in gymnasium.
'''

import gymnasium as gym
from gymnasium.envs.mujoco.mujoco_env import MujocoEnv
import os
from gymnasium import spaces
import numpy as np

class SimpleRobotEnv(MujocoEnv):
    metadata = {
        "render_modes": ["human", "rgb_array", "depth_array"],
        "render_fps": 1000,
    }

    def __init__(self, **kwargs):
        current_dir = os.path.dirname(os.path.abspath(__file__))
        xml_file = os.path.join(current_dir, "robot/robot.mjcf")
        
        # Define observation space before calling super().__init__
        observation_space = spaces.Box(
            low=-np.inf,
            high=np.inf,
            shape=(1,),  # Adjust shape based on your MJCF model's DOF
            dtype=np.float64
        )

        super().__init__(
            model_path=xml_file,
            frame_skip=1,
            observation_space=observation_space,
            **kwargs
        )

    def step(self, action):
        # Simple implementation just for visualization
        self.do_simulation(action, self.frame_skip)
        observation = self._get_obs()
        reward = 0.0
        terminated = False
        truncated = False
        info = {}
        return observation, reward, terminated, truncated, info

    def reset_model(self):
        self.set_state(self.init_qpos, self.init_qvel)
        return self._get_obs()
    
    def _get_obs(self):
        return np.array([0.0])  # Return minimal observation for visualization

# Register and create environment
gym.register(
    id="SimpleRobot-v0",
    entry_point=SimpleRobotEnv,
    max_episode_steps=1000,
)

# Create and run visualization
env = gym.make("SimpleRobot-v0", render_mode="human")
env.reset()

try:
    while True:
        env.step(env.action_space.sample())
        env.render()
except KeyboardInterrupt:
    env.close()