from gz_biped_gym import BalanceEnv
import numpy as np
import time

import python.model as models

"""
TODO

static balance env for the blackbird first
"""

env = BalanceEnv.BalanceBird(world_path="world/empty.world", render_mode="human")
i = 0
while True:
    #action = np.random.uniform(low=-50.0, high=50.0, size=(10,))
    action = np.zeros(10)
    action[2] = 20.0
    state, reward, terminal, _ = env.step(action)
    if (terminal):
        print(f"reached a terminal at idx {i}. resetting...")
        env.reset()
    i+=1
    # time.sleep(.1)