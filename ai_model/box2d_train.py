"""
test_agent.py

test this agent on a 
bipedalwalker-v3
env to ensure correctness
of the model
"""
import gymnasium as gym
import numpy as np
import time
from python.agent import BlackbirdDDPG

env = gym.make("BipedalWalker-v3", render_mode="human")
action_size = env.action_space.shape[0]
state_size = env.observation_space.shape[0]
agent = BlackbirdDDPG(env, state_size, action_size)

print("here's action low: ", env.action_space.low)

num_iters = 400000
episode_steps = 0
num_episodes = 0
state, info = env.reset()

for i in range(num_iters):
    if (i < 1000):
        action = env.action_space.sample()
    else:
        action = agent.select_action(agent.s_t)

    state, reward, terminal, truncated, info = env.step(action)
    agent.add_experience(reward, state, terminal)

    if (terminal or truncated or episode_steps == 5000):
        env.reset()
        episode_steps = 0
        num_episodes += 1

    i+=1
    episode_steps += 1

    
    if (i > 3000):
        agent.optimize()

agent.save_model()
