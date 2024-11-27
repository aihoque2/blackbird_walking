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

env = gym.make("BipedalWalker-v3", render_mode="human")
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

