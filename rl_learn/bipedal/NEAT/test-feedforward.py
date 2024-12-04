import os
import pickle
import neat
import gymnasium

# load the winner
with open('winner-feedforward', 'rb') as f:
    c = pickle.load(f)

print('Loaded genome:')
print(c)

# Load the config file, which is assumed to live in
# the same directory as this script.
local_dir = os.path.dirname(__file__)
config_path = os.path.join(local_dir, 'config-feedforward')
config = neat.Config(neat.DefaultGenome, neat.DefaultReproduction,
                     neat.DefaultSpeciesSet, neat.DefaultStagnation,
                     config_path)

net = neat.nn.FeedForwardNetwork.create(c, config)
env = gymnasium.make("BipedalWalker-v3", render_mode="human")
observation, info = env.reset()
terminated = False
truncated = False 

print()
print("Initial conditions:")
print("Cart Position     = {0:.4f}".format(observation[0]))
print("Cart Velocity     = {0:.4f}".format(observation[1]))
print("Pole Angle       = {0:.4f}".format(observation[2]))
print("Pole Ang Velocity = {0:.4f}".format(observation[3]))
print()


while not (terminated or truncated):
        action = net.activate(observation)
        observation, reward, terminated, truncated, info = env.step(action)
        env.render()
