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
ACTOR_RATE = 0.0001
CRITIC_RATE = 0.001

env = BlackbirdEnv.BlackbirdGazebo(world_path="world/empty.world", render_mode="human")
state_size = 35
action_size = 10

agent = BlackbirdDDPG(env, state_size, action_size, 50.0, prate=ACTOR_RATE, rate=CRITIC_RATE)

sum_reward = 0.0
num_episodes = 0
episode_steps = 0
num_iters = int(400000) # 700 episodes??

state, reward, terminal, _ = env.step([0.0]*10)

for i in tqdm.tqdm(range(num_iters)):
    if (i < 3000):
        action = agent.random_action()
    else:
        action = agent.select_action(agent.s_t)

    # @ beginning of episode, let robot stay still until it "lands"
    if (episode_steps < 500 and (not state[33] and not state[34])):
        action = np.zeros(10)

    state, reward, terminal, _ = env.step(action)
    agent.add_experience(reward, state, terminal)
    sum_reward += reward

    if (i%1000 == 0):
        print(f"reached itereation {i}/{num_iters}")

    if (terminal or episode_steps >= 2000):
        print(f"reached a terminal at idx {i}.\ncumulative reward:{sum_reward} \nresetting...")
        env.reset()
        agent.reset(state)
        episode_steps = 0
        num_episodes += 1
        sum_reward = 0
        print(f"\n end of an episode. begin epsiode {num_episodes}")

    if (i > 3000):
        print("optimization step")
        agent.optimize()

    # if (i%250000 == 0):
    #     agent.save_model()

    
    # debug
    # if (agent.a_t.is_cuda):
    #     print(f"agent a_t is_cuda at i={i}")
    #     break
    episode_steps += 1

agent.save_model("models")
print ("training complete!")