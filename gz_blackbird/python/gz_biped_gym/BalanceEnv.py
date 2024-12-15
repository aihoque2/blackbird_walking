import gym
import numpy as np
from gym import spaces
import sys
import blackbird_rl
import os
import importlib.resources
import sys
from pathlib import Path


class BalanceBird(gym.Env):
    metadata = {"render_modes": ["human", "rgb_array"], "render_fps": 4}

    def __init__(self,  world_path, render_mode=None):

        set_gui = False
        if render_mode is not None:
            set_gui = True
        else:
            set_gui = False


        self.sim = blackbird_rl.TrainSimulator(set_gui, world_path)
        obs_low = np.full(32, -np.inf)  # -inf for each element
        obs_high = np.full(32, np.inf)  # inf for each element

        self.observation_space = spaces.Box(low=obs_low, high=obs_high, dtype=np.float64)

        axn_low = np.full(10, -50.0)
        axn_hi = np.full(10, 50.0)
        self.action_space = spaces.Box(low=axn_low, high=axn_hi, dtype=np.float64)
        self.steps = 0
        
        # hyperparameters
        self.Z_WEIGHT = 80.0
        self.ROT_WEIGHT = 200.0
        self.POWER_WEIGHT = 0.05 
        self.SMOOTH_WEIGHT = 20. # joint velocity smoothing

    
    def det_terminal(self):
        """
        determine terminal
        """
        return self.sim.is_terminal() # terminal for now is if robot's chassis contacts the ground

    def reset(self):
        self.sim.reset_sim()
        self.steps = 0
        state = self.sim.get_state()
        info = {"pose": [state[0], state[1], state[2]], 
                "orientation": [state[3], state[4], state[5]]}
        return np.array(state, dtype=np.float32), info

    def step(self, action):
        
        self.sim.step(action)
        state = self.sim.get_state()

        torso_contacted = state[32]
        l_contacted = state[33] # 1.0 if contacted 0.0 otherwise
        r_contacted = state[34] # 1.0 if contacted 0.0 otherwise
        legs_contacted = l_contacted and r_contacted
        
        # KEEP THESE. COMMENT/UNCOMMENT ACCORDINGLY
        # print(f"here's l_contacted: {l_contacted}")
        # print(f"here's r_contacted: {r_contacted}")
        # print()

        pose_z = state[2] # height of the robot's position
        valid_height = (0.50 <= pose_z <= 1.05)

        roll, pitch, yaw = state[3], state[4], state[5]     

        power = 0.0
        for i in range(len(action)):
            power += action[i] * state[i+22]

        reward = -self.ROT_WEIGHT*(np.abs(roll) + np.abs(pitch) + np.abs(yaw))\
              - self.POWER_WEIGHT*power + legs_contacted*valid_height*self.Z_WEIGHT*pose_z\
              -self.SMOOTH_WEIGHT*sum([state[i] for i in range(22, 32)])

        terminal = self.det_terminal()
        if (terminal):
            reward = -100000.0 # terminal_penalty

        self.steps += 1
        return np.array(state, dtype=np.float32), reward, self.det_terminal(), {}

if __name__ == '__main__':
    env = BalanceBird(render_mode="human")

    for i in range(10000):
        action = np.random.uniform(low=-50.0, high=50.0, size=(10,))
        # action = np.zeros(10)
        state, reward, terminal, _ = env.step(action)
        if (terminal):
            print(f"reached a terminal at idx {i}. resetting...")
            env.reset()