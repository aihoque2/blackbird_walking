import gym
import numpy as np
from gym import spaces
import sys
import blackbird_rl
import os
import pkg_resources
import sys



class BlackbirdGazebo(gym.Env):
    metadata = {"render_modes": ["human", "rgb_array"], "render_fps": 4}

    def __init__(self, render_mode=None):

        set_gui = False
        if render_mode is not None:
            set_gui = True
        else:
            set_gui = False

        # Get the full path to `empty.world` in `gz_biped_gym.world`
        pkg_path = pkg_resources.resource_filename('gz_biped_gym', '/')
        print(pkg_path)
        sys.path.append(pkg_path)
        self.sim = blackbird_rl.TrainSimulator(set_gui, pkg_path+'world/empty.world')
        obs_low = np.full(32, -np.inf)  # -inf for each element
        obs_high = np.full(32, np.inf)  # inf for each element

        self.observation_space = spaces.Box(low=obs_low, high=obs_high, dtype=np.float64)

        axn_low = np.full(10, -50.0)
        axn_hi = np.full(10, 50.0)
        self.action_space = spaces.Box(low=axn_low, high=axn_hi, dtype=np.float64)
        self.steps = 0
        
        # hyperparameters
        self.Y_WEIGHT = -100.0
        self.POWER_WEIGHT = 0.05
        self.Z_WEIGHT = 50.0
    
    def det_terminal(self):
        """
        determine terminal
        """

        if (self.steps >= 10000):
            return True
        if (self.sim.is_terminal()):
            return True
        return False

    def reset(self):
        self.sim.reset_sim()
        self.steps = 0
        state = self.sim.get_state()
        info = {"pose": {state[0], state[1], state[2]}}
        return state, info

    def step(self, action):
        
        self.sim.step(action)
        state = self.sim.get_state()

        l_contacted = state[33]
        r_contacted = state[34]
        legs_contacted = l_contacted or r_contacted

        print(f"here's l_contacted: {l_contacted}")
        print(f"here's r_contacted: {r_contacted}")
        print()

        pose_y = state[1]
        pose_z = state[2]
        
        power = 0.0
        for i in range(len(action)):
            power += action[i] * state[i+12]

        reward = self.Y_WEIGHT*pose_y - self.POWER_WEIGHT*power + legs_contacted*self.Z_WEIGHT*pose_z

        self.steps += 1
        return np.array(state, dtype=np.float32), reward, self.det_terminal(), {}
