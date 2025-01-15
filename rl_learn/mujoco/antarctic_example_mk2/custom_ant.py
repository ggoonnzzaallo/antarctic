from gymnasium.envs.mujoco.ant_v5 import AntEnv
import numpy as np
import gymnasium as gym

class ForwardFacingAntEnv(AntEnv):
    def __init__(
        self,
        xml_file='./scene.xml',
        forward_reward_weight=1,
        ctrl_cost_weight=0.05,
        contact_cost_weight=5e-4,
        healthy_reward=1,
        main_body=1,
        healthy_z_range=(0.15, 0.25),
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
        
        # Override the observation space
        obs_size = 31  # 12 + 12 + 4 + 3
        self.observation_space = gym.spaces.Box(
            low=-np.inf, high=np.inf, shape=(obs_size,), dtype=np.float64
        )
        
        # Normalized action space (-1 to 1)
        self.action_space = gym.spaces.Box(
            low=-1.0,
            high=1.0,
            shape=(12,),  # 12 actuators
            dtype=np.float64
        )
        
    def step(self, action):
        # Print action values
        print(f"\nAction values: {action}")
        
        # Scale normalized actions (-1 to 1) to joint angles (0 to π)
        scaled_action = 0.5 * (action + 1.0) * np.pi  # Maps [-1, 1] to [0, π]
        print(f"Scaled action values: {scaled_action}")
        
        observation, reward, terminated, truncated, info = super().step(scaled_action)
        
        # Get and print the height (z-coordinate)
        height = self.data.qpos[2]  # Index 2 is the z-coordinate
        print(f"Ant height: {height:.3f}m")
        
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
        
        # Get orientation quaternion from IMU sensor
        orientation_quat = self.data.sensordata[:4]  # First 4 values are the quaternion
        print(f"Raw quaternion: {orientation_quat}")  # Debug print
        
        # Convert quaternion to Euler angles (roll, pitch, yaw)
        def quat_to_euler(q):
            # MuJoCo quaternion order is [w, x, y, z]
            w, x, y, z = q  # Changed from x,y,z,w to w,x,y,z
            
            # roll (x-axis rotation)
            sinr_cosp = 2.0 * (w * x + y * z)
            cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
            roll = np.arctan2(sinr_cosp, cosr_cosp)
            
            # pitch (y-axis rotation)
            sinp = 2.0 * (w * y - z * x)
            pitch = np.arcsin(sinp)
            
            # yaw (z-axis rotation)
            siny_cosp = 2.0 * (w * z + x * y)
            cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
            yaw = np.arctan2(siny_cosp, cosy_cosp)
            
            return np.array([roll, pitch, yaw])
        
        euler_angles = quat_to_euler(orientation_quat)
        roll, pitch, yaw = euler_angles
        print(f"Euler angles (degrees): roll={np.degrees(roll):.1f}°, pitch={np.degrees(pitch):.1f}°, yaw={np.degrees(yaw):.1f}°")  # Debug print
        
        # Define maximum allowed tilt (in radians)
        max_tilt = np.pi/3  # Increased to 60 degrees for testing
        
        # Check if robot has tilted too much
        terminated = terminated or (abs(roll) > max_tilt or abs(pitch) > max_tilt)
        
        if terminated:
            print(f"Terminating due to excessive tilt: roll={np.degrees(roll):.1f}°, pitch={np.degrees(pitch):.1f}°")  # Debug print
            reward = -100
        
        return observation, modified_reward, terminated, truncated, info 

    def _get_obs(self):
        # Get actuator/joint positions (12 values)
        position = self.data.qpos[7:19]  # Skip root position (3) and orientation (4)
        
        # Get joint velocities (12 values)
        velocity = self.data.qvel[6:18]  # Skip root linear (3) and angular (3) velocities
        
        # Get orientation quaternion (4 values) and angular velocity (3 values)
        orientation = self.data.qpos[3:7]  # Root orientation quaternion
        angular_vel = self.data.qvel[3:6]  # Root angular velocity

        # Combine all observations
        observations = np.concatenate([
            position,     # 12 values
            velocity,     # 12 values
            orientation,  # 4 values
            angular_vel   # 3 values
        ])
        
        return observations 

    def reset_model(self):
        noise_low = -self._reset_noise_scale
        noise_high = self._reset_noise_scale

        qpos = self.init_qpos + self.np_random.uniform(
            low=noise_low, high=noise_high, size=self.model.nq
        )
        qvel = self.init_qvel + self.np_random.uniform(
            low=noise_low, high=noise_high, size=self.model.nv
        )

        # Set initial position and orientation
        qpos[0:3] = [0, 0, 0.185]  # x, y, z position
        qpos[3:7] = [1, 0, 0, 0]  # quaternion orientation (w, x, y, z)

        # Use the same joint positions as in mujoco_script.py
        default_joint_angles = np.array([
            1.57,   # dof_shoulder_front_left
            0.0,    # dof_hip_front_left
            0.85,    # dof_knee_front_left
            1.57,   # dof_shoulder_front_right
            0.0,    # dof_hip_front_right
            2.29,   # dof_knee_front_right
            1.57,  # dof_shoulder_rear_right
            1.57,   # dof_hip_rear_right
            2.29,   # dof_knee_rear_right
            1.57,   # dof_shoulder_rear_left
            0.0,    # dof_hip_rear_left
            0.85     # dof_knee_rear_left
        ])
        qpos[7:19] = default_joint_angles

        self.set_state(qpos, qvel)
        return self._get_obs() 