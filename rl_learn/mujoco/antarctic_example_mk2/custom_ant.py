from gymnasium.envs.mujoco.ant_v5 import AntEnv
import numpy as np

class ForwardFacingAntEnv(AntEnv):
    def __init__(
        self,
        xml_file='./scene.xml',
        forward_reward_weight=1,
        ctrl_cost_weight=0.05,
        contact_cost_weight=5e-4,
        healthy_reward=1,
        main_body=1,
        healthy_z_range=(0, 0.75),
        include_cfrc_ext_in_observation=False,
        exclude_current_positions_from_observation=False,
        reset_noise_scale=0.1,
        frame_skip=25,
        max_episode_steps=1000,
        **kwargs
    ):
        super().__init__(
            xml_file=xml_file,
            forward_reward_weight=forward_reward_weight,
            ctrl_cost_weight=ctrl_cost_weight,
            contact_cost_weight=contact_cost_weight,
            healthy_reward=healthy_reward,
            terminate_when_unhealthy=True,
            healthy_z_range=healthy_z_range,
            include_cfrc_ext_in_observation=include_cfrc_ext_in_observation,
            exclude_current_positions_from_observation=exclude_current_positions_from_observation,
            reset_noise_scale=reset_noise_scale,
            frame_skip=frame_skip,
            **kwargs
        )
        
    def step(self, action):
        observation, reward, terminated, truncated, info = super().step(action)
        
        # Get the ant's orientation (using 'root' instead of 'torso')
        main_body_id = self.data.body('root').id
        rotation_matrix = self.data.xmat[main_body_id].reshape(3, 3)
        forward_direction = rotation_matrix[:2, 0]
        
        # Get velocity direction
        xy_velocity = self.data.qvel[:2]
        velocity_magnitude = np.linalg.norm(xy_velocity)
        if velocity_magnitude > 0:
            velocity_direction = xy_velocity / velocity_magnitude
        else:
            velocity_direction = np.array([1, 0])
        
        # Calculate alignment reward
        alignment = np.dot(forward_direction, velocity_direction)
        alignment_reward = alignment * velocity_magnitude * 0.5
        
        modified_reward = reward + alignment_reward
        info['alignment_reward'] = alignment_reward
        
        return observation, modified_reward, terminated, truncated, info 