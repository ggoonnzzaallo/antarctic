import gymnasium

env = gymnasium.make("BipedalWalker-v3", render_mode="human")

while True:

    observation, info = env.reset()
    terminated = False
    truncated = False
        
    while not (terminated or truncated):
        action = env.action_space.sample()
        observation, reward, terminated, truncated, info = env.step(action)
        print(observation)
        env.render()

    # action = env.action_space.sample()
    # observation, reward, terminated, truncated, info = env.step(action)
    # env.render()


env.close()


