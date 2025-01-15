'''
This script is used to evaluate the model.

To run this you must specify the algorithm and model path at a minimum. You can also specify other parameters like eval-episodes, learning-rate, batch-size, n-steps, gamma, etc.

For example, you can run:
python evaluate.py --algo PPO --model-path models/PPO/best_model/best_model
'''


import os
import gymnasium as gym
import argparse
from stable_baselines3 import PPO, A2C, SAC, TD3
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize
import numpy as np
from custom_ant import ForwardFacingAntEnv

# Dictionary of available algorithms
ALGORITHMS = {
    "PPO": PPO,
    "A2C": A2C,
    "SAC": SAC,
    "TD3": TD3
}

def make_env(render_mode=None):
    """Create the forward-facing Ant environment"""
    env = ForwardFacingAntEnv(
        xml_file='./scene.xml',
        forward_reward_weight=1,
        ctrl_cost_weight=0.05,
        contact_cost_weight=5e-4,
        healthy_reward=1,
        main_body=1,
        healthy_z_range=(0.15, 0.25),
        include_cfrc_ext_in_observation=True,
        exclude_current_positions_from_observation=False,
        reset_noise_scale=0.1,
        frame_skip=25,
        render_mode=render_mode
    )
    return env

def evaluate_model(algo_name, model_path, episodes=5):
    """Evaluate a trained model with rendering"""
    # Load the environment with rendering
    env = make_env(render_mode="human")
    env = DummyVecEnv([lambda: env])
    
    # Load the stats from training
    stats_path = os.path.join(os.path.dirname(model_path), "vec_normalize.pkl")
    if os.path.exists(stats_path):
        env = VecNormalize.load(stats_path, env)
        # Don't update the normalizer during evaluation
        env.training = False
        env.norm_reward = False
    
    # Load the model
    try:
        model = ALGORITHMS[algo_name].load(model_path, env=env)  # Pass env to ensure compatibility
    except FileNotFoundError:
        print(f"\nERROR: Could not find model at {model_path}")
        return
    except Exception as e:
        print(f"\nERROR: Failed to load model: {str(e)}")
        return

    print("\nStarting evaluation...")
    for episode in range(episodes):
        obs = env.reset()
        done = False
        total_reward = 0
        step_count = 0
        
        while not done:
            action, _ = model.predict(obs, deterministic=True)
            obs, reward, done, info = env.step(action)
            if isinstance(done, tuple):
                done = done[0]
            total_reward += reward[0]
            step_count += 1
            
            if done:
                print(f"\nEpisode {episode + 1}:")
                print(f"- Total reward: {total_reward:.2f}")
                print(f"- Steps: {step_count}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Evaluate a trained model on Ant-v5')
    parser.add_argument('--algo', type=str, choices=['PPO', 'A2C', 'SAC', 'TD3'],
                        required=True, help='Algorithm to evaluate')
    parser.add_argument('--model-path', type=str,
                        help='Path to the model file. If not provided, will use best_model from default location')
    parser.add_argument('--checkpoint', type=int,
                        help='Checkpoint number to evaluate (alternative to --model-path)')
    parser.add_argument('--episodes', type=int, default=5,
                        help='Number of episodes to evaluate')
    
    args = parser.parse_args()
    
    # Determine the model path
    if args.checkpoint is not None:
        args.model_path = f"models/{args.algo}/model_{args.checkpoint}_steps"
    elif args.model_path is None:
        args.model_path = f"models/{args.algo}/best_model/best_model"
    
    evaluate_model(args.algo, args.model_path, episodes=args.episodes)
