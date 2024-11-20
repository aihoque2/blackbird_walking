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
    action = agent.random_action()
    #action = np.zeros(10)
    state, reward, terminal, _ = env.step(action)
    if (terminal):
        print(f"reached a terminal at idx {i}. resetting...")
        env.reset()
        agent.reset(state)
    i+=1