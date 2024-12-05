import os
import gymnasium as gym
import argparse
from stable_baselines3 import PPO, A2C, SAC, TD3
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize

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

def evaluate_model(algo_name, model_path, episodes=5):
    """Evaluate a trained model with rendering"""
    # Load the environment with rendering
    env = make_env(render_mode="human")
    env = DummyVecEnv([lambda: env])
    
    # Load the model
    try:
        model = ALGORITHMS[algo_name].load(model_path)
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
    parser = argparse.ArgumentParser(description='Evaluate a trained model on BipedalWalker-v3')
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
