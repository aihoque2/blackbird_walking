from gz_biped_gym import BlackbirdEnv
import numpy as np
import time
import tqdm

import python.model as models
from python.agent import BlackbirdDDPG


"""
train_agent.py
script to run 1 batch
of training a reinforcement model
"""


env = BlackbirdEnv.BlackbirdGazebo(world_path="world/empty.world", render_mode="human")
state_size = 35
action_size = 10

agent = BlackbirdDDPG(env, state_size, action_size)

sum_reward = 0.0
num_episodes = 0
num_iters = int(7e6) # 700 episodes??

for i in tqdm.tqdm(range(num_iters)):
    if (i < 1000):
        action = agent.random_action()
    else:
        action = agent.select_action(agent.s_t)

    next_state, reward, terminal, _ = env.step(action)
    agent.add_experience(reward, next_state, terminal)
    sum_reward += reward

    if (terminal):
        print(f"reached a terminal at idx {i}. resetting...")
        env.reset()
        agent.reset(next_state)
        num_episodes = 0
        sum_reward = 0
        print(f"end of an episode. begin epsiode {num_episodes}")

    if (len(agent.memory) > 3000):
        agent.optimize()
    i+=1

agent.save_model()