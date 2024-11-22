from gz_biped_gym import BlackbirdEnv
import numpy as np
import time

import python.model as models
from python.agent import BlackbirdDDPG

env = BlackbirdEnv.BlackbirdGazebo(world_path="world/empty.world", render_mode="human")
state_size = 35
action_size = 10

agent = BlackbirdDDPG(env, state_size, action_size)

i = 0
while True:
    if (i < 1000):
        action = agent.random_action()
    else:
        action = agent.select_action(agent.s_t)

    next_state, reward, terminal, _ = env.step(action)
    agent.add_experience(reward, next_state, terminal)

    if (terminal):
        print(f"reached a terminal at idx {i}. resetting...")
        env.reset()
        agent.reset(next_state)

    if (len(agent.memory) > 3000):
        agent.optimize()
    i+=1