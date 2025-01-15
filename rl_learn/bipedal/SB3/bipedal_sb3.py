import os
import gymnasium as gym
import argparse
from stable_baselines3 import PPO, A2C, SAC, TD3
from stable_baselines3.common.callbacks import EvalCallback, CheckpointCallback, BaseCallback
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize
import subprocess
import webbrowser
import time
from threading import Thread

# Dictionary of available algorithms
ALGORITHMS = {
    "PPO": PPO,
    "A2C": A2C,
    "SAC": SAC,
    "TD3": TD3
}

def make_env(render_mode=None):
    """Create the BipedalWalker environment"""
    env = gym.make("BipedalWalker-v3", render_mode=render_mode)
    return env

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

def launch_tensorboard(log_dir):
    """Launch Tensorboard in a separate thread"""
    def run_tensorboard():
        subprocess.run(['tensorboard', '--logdir', log_dir, '--port', '6006'])

    # Start Tensorboard in a separate thread
    tensorboard_thread = Thread(target=run_tensorboard, daemon=True)
    tensorboard_thread.start()
    
    # Wait a moment for Tensorboard to start
    time.sleep(3)
    
    # Open the default web browser
    webbrowser.open('http://localhost:6006')

def train_model(algo_name="PPO", total_timesteps=1_000_000):
    """Train a model using the specified algorithm"""
    # Create logs directory
    log_dir = f"logs/{algo_name}"
    os.makedirs(log_dir, exist_ok=True)
    
    # Launch Tensorboard
    launch_tensorboard(log_dir)
    
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

    # Callback
    eval_callback = EvalCallback(
        eval_env=eval_env,
        best_model_save_path=best_model_dir,
        log_path=log_dir,
        eval_freq=10000,
        deterministic=True,
        render=False,
        n_eval_episodes=5
    )

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
        callback=eval_callback
    )

def evaluate_model(algo_name, model_path, episodes=5):
    """Evaluate a trained model with rendering"""
    # Load the environment with rendering
    env = make_env(render_mode="human")
    env = DummyVecEnv([lambda: env])
    
    # Load the saved normalization parameters
    env = VecNormalize.load(
        f"models/{algo_name}/vec_normalize.pkl", 
        env
    )
    env.training = False
    env.norm_reward = False

    # Load the model
    model = ALGORITHMS[algo_name].load(model_path)

    for episode in range(episodes):
        obs = env.reset()
        done = False
        total_reward = 0
        
        while not done:
            action, _ = model.predict(obs, deterministic=True)
            obs, reward, done, info = env.step(action)
            total_reward += reward[0]
            
        print(f"Episode {episode + 1} reward: {total_reward}")

if __name__ == "__main__":
    # Set up argument parser
    parser = argparse.ArgumentParser(description='Train and evaluate a model on BipedalWalker-v3')
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
    
    # Get default parameters for the chosen algorithm
    algo_params = get_algorithm_params(args.algo)
    
    # Override with any provided command-line arguments
    if args.learning_rate is not None:
        algo_params['learning_rate'] = args.learning_rate
    if args.batch_size is not None:
        algo_params['batch_size'] = args.batch_size
    if args.n_steps is not None and args.algo in ['PPO', 'A2C']:
        algo_params['n_steps'] = args.n_steps
    if args.gamma is not None:
        algo_params['gamma'] = args.gamma
    
    # Train the model
    train_model(args.algo, total_timesteps=args.timesteps)
    
    # Evaluate the best model
    evaluate_model(
        args.algo,
        f"models/{args.algo}/best_model/best_model",
        episodes=args.eval_episodes
    )
