#Following: https://gymnasium.farama.org/tutorials/gymnasium_basics/load_quadruped_model/

'''
This script is used to train the model.

To run this you must specify the algorithm and timesteps at a minimum. You can also specify other parameters like eval-episodes, learning-rate, batch-size, n-steps, gamma, etc.

For example, you can run:
python train.py --algo PPO --timesteps 1000000
'''

import gymnasium as gym
import numpy as np
import os
import argparse
from stable_baselines3 import PPO, A2C, SAC, TD3
from stable_baselines3.common.callbacks import EvalCallback, CheckpointCallback, CallbackList
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import DummyVecEnv
from custom_ant import ForwardFacingAntEnv

# Dictionary of available algorithms
ALGORITHMS = {
    "PPO": PPO,
    "A2C": A2C,
    "SAC": SAC,
    "TD3": TD3
}

def get_algorithm_params(algo_name):
    """Get algorithm-specific parameters"""
    if algo_name == "PPO":
        return {
            "n_steps": 2048,          # Number of steps to run for each environment per update
            "batch_size": 64,         # Minibatch size for each gradient update
            "n_epochs": 10,           # Number of epochs when optimizing the surrogate loss
            "learning_rate": 3e-4,    # Learning rate
            "ent_coef": 0.0,         # Entropy coefficient for exploration
            "clip_range": 0.2,        # Clipping parameter for PPO
            "gae_lambda": 0.95,       # Factor for trade-off of bias vs variance for GAE
            "gamma": 0.99,            # Discount factor
        }
    elif algo_name == "A2C":
        return {
            "n_steps": 5,            # Number of steps to run for each environment per update
            "ent_coef": 0.0,         # Entropy coefficient for exploration
            "learning_rate": 7e-4,    # Learning rate
            "gamma": 0.99,           # Discount factor
            "gae_lambda": 1.0,       # Factor for trade-off of bias vs variance for GAE
            "rms_prop_eps": 1e-5,    # RMSprop epsilon
        }
    elif algo_name == "SAC":
        return {
            "learning_rate": 3e-4,    # Learning rate
            "buffer_size": 1000000,   # Size of the replay buffer
            "learning_starts": 100,   # Number of steps before learning starts
            "batch_size": 256,        # Minibatch size for each gradient update
            "tau": 0.005,            # Target smoothing coefficient
            "gamma": 0.99,           # Discount factor
            "train_freq": 1,         # Update the model every train_freq steps
            "gradient_steps": 1,      # Gradient steps per update
            "ent_coef": "auto",      # Entropy coefficient (auto-adjusted)
        }
    elif algo_name == "TD3":
        return {
            "learning_rate": 3e-4,    # Learning rate
            "buffer_size": 1000000,   # Size of the replay buffer
            "learning_starts": 100,   # Number of steps before learning starts
            "batch_size": 100,        # Minibatch size for each gradient update
            "tau": 0.005,            # Target smoothing coefficient
            "gamma": 0.99,           # Discount factor
            "train_freq": 1,         # Update the model every train_freq steps
            "gradient_steps": 1,      # Gradient steps per update
            "policy_delay": 2,       # Delay for policy updates
        }
    return {}
    
def make_env(render_mode="human"):
    """Create the forward-facing Ant environment"""
    env = ForwardFacingAntEnv(
        xml_file='./scene.xml',
        forward_reward_weight=1,
        ctrl_cost_weight=0.05,
        contact_cost_weight=5e-4,
        healthy_reward=1,
        main_body=1,
        healthy_z_range=(0, 0.75),
        include_cfrc_ext_in_observation=True,
        exclude_current_positions_from_observation=False,
        reset_noise_scale=0.1,
        frame_skip=25,
        render_mode=render_mode
    )
    print("Observation Space:", env.observation_space.shape)
    return env


# def make_env(render_mode="human"):
#     """Create the default Ant-v5 environment"""
#     env = gym.make(
#         'Ant-v5',
#         render_mode=render_mode,
#         terminate_when_unhealthy=True,  # Default Ant-v5 setting
#         healthy_z_range=(0.2, 1.0),     # Default Ant-v5 setting
#     )
#     print("Observation Space:", env.observation_space.shape)
#     return env


def train_model(algo_name="PPO", total_timesteps=1_000_000):
    """Train a model using the specified algorithm"""
    # Create logs directory
    log_dir = f"logs/{algo_name}"
    os.makedirs(log_dir, exist_ok=True)
    
    # Create model directory
    models_dir = f"models/{algo_name}"
    best_model_dir = os.path.join(models_dir, "best_model")
    os.makedirs(best_model_dir, exist_ok=True)

    # Create and wrap the environment
    env = make_env()
    env = Monitor(env, log_dir)
    env = DummyVecEnv([lambda: env])

    # Create evaluation environment
    eval_env = make_env()
    eval_env = Monitor(eval_env, log_dir)
    eval_env = DummyVecEnv([lambda: eval_env])

    # Callback setup
    checkpoint_callback = CheckpointCallback(
        save_freq=10000, #used to be 10000
        save_path=models_dir,
        name_prefix="model",
        save_replay_buffer=True,
        save_vecnormalize=True
    )

    eval_callback = EvalCallback(
        eval_env=eval_env,
        best_model_save_path=best_model_dir,
        log_path=log_dir,
        eval_freq=10000, #used to be 10000
        deterministic=True,
        render=False,
        n_eval_episodes=5
    )

    callbacks = CallbackList([checkpoint_callback, eval_callback])

    # Train the model
    model = ALGORITHMS[algo_name](
        "MlpPolicy",
        env,
        verbose=1,
        tensorboard_log=log_dir,
        **get_algorithm_params(algo_name)
    )

    model.learn(
        total_timesteps=total_timesteps,
        callback=callbacks
    )

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Train and evaluate a model on Ant-v5')
    parser.add_argument('--algo', type=str, choices=['PPO', 'A2C', 'SAC', 'TD3'],
                        default='PPO', help='Algorithm to use for training')
    parser.add_argument('--timesteps', type=int, default=1_000_000,
                        help='Total timesteps for training')
    parser.add_argument('--eval-episodes', type=int, default=5,
                        help='Number of episodes for evaluation')
    
    # Add hyperparameter tuning arguments
    parser.add_argument('--learning-rate', type=float, help='Learning rate')
    parser.add_argument('--batch-size', type=int, help='Batch size')
    parser.add_argument('--n-steps', type=int, help='Number of steps per update (for PPO/A2C)')
    parser.add_argument('--gamma', type=float, help='Discount factor')
    
    args = parser.parse_args()
    
    # Train the model
    train_model(args.algo, total_timesteps=args.timesteps)
