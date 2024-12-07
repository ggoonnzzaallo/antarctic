import gymnasium as gym

# Create the environment
env = gym.make("Walker2d-v5", render_mode="human")

# Get initial observation
observation, info = env.reset()

# Run the environment
for _ in range(1000):  # Run for 1000 steps
    # Sample a random action
    action = env.action_space.sample()
    
    # Apply the action and get the next state
    observation, reward, terminated, truncated, info = env.step(action)
    
    # Check if episode is done
    if terminated or truncated:
        observation, info = env.reset()

# Clean up
env.close()
