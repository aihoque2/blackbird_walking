from gz_biped_gym import BlackbirdEnv
import numpy as np
import time

env = BlackbirdEnv.BlackbirdGazebo(render_mode="human")

for i in range(10000):
    #action = np.random.uniform(low=-50.0, high=50.0, size=(10,))
    action = np.zeros(10)
    state, reward, terminal, _ = env.step(action)
    if (terminal):
        print(f"reached a terminal at idx {i}. resetting...")
        env.reset()