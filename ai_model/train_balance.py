from gz_biped_gym import BalanceEnv
import numpy as np
import time
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
agent = BlackbirdDDPG(env, state_size, action_size, 1.0, prate=ACTOR_RATE, rate=CRITIC_RATE)
# agent.load_weights('models')

num_iters = 250000
episode_steps = 0
num_episodes = 0
sum_reward = 0.0
state, info = env.reset()
i=0

for _ in tqdm.tqdm(range(num_iters)):
    # action = np.random.uniform(low=-50.0, high=50.0, size=(10,))
    if (i < 10000):
        action = agent.random_action()

    else:
        action = agent.select_action(agent.s_t)

    state, reward, terminal, _ = env.step(action)
    sum_reward += reward

    agent.add_experience(reward, state, terminal)

    if (terminal or episode_steps == 10000):
        print(f"reached a terminal at idx {i}. resetting...")
        reward_str = f"reward at episode {num_episodes}: {sum_reward}"
        env.reset()
        episode_steps = 0
        sum_reward = 0.0
        num_episodes += 1
    
    if (i > 10000):
        agent.optimize()

    i+=1
    episode_steps += 1

agent.save_model("models")
    # time.sleep(.1)