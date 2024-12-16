"""
test_balance.py
"""
from gz_biped_gym import BalanceEnv
import numpy as np
import time
import torch
import gym

from python.agent import BlackbirdDDPG
import tqdm as tqdm

import torch
import torch.multiprocessing
torch.multiprocessing.set_sharing_strategy('file_system')

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

ACTOR_RATE = 0.0001
CRITIC_RATE = 0.001


"""
TODO

static balance env for the blackbird first
"""

env = BalanceEnv.BalanceBird(world_path="world/empty.world", render_mode="human")

action_size = 10
state_size = 35
agent = BlackbirdDDPG(env, state_size, action_size, 10.0, prate=ACTOR_RATE, rate=CRITIC_RATE)
agent.load_weights('models')

episode_steps = 0
num_episodes = 0
sum_reward = 0.0
state, info = env.reset()
i=0

for _ in range(100000):

    action = agent.select_action(torch.tensor(state))

    state, reward, terminal, _ = env.step(action)
    sum_reward += reward

    if (terminal or episode_steps == 3000):
        print(f"reached a terminal at idx {i}. resetting...")
        reward_str = f"reward at episode {num_episodes}: {sum_reward}"
        env.reset()
        episode_steps = 0
        sum_reward = 0.0
        num_episodes += 1
    
    i+=1
    episode_steps += 1
    time.sleep(0.001)