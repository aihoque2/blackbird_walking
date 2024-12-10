import gym
import numpy as np
from gym import spaces
import sys
import blackbird_rl
import os
import importlib.resources
import sys
from pathlib import Path


class BlackbirdGazebo(gym.Env):
    metadata = {"render_modes": ["human", "rgb_array"], "render_fps": 4}

    def __init__(self,  world_path, render_mode=None):

        set_gui = False
        if render_mode is not None:
            set_gui = True
        else:
            set_gui = False


        # # Step 1: Get the absolute path to `blackbird.sdf` using `importlib.resources`
        # with importlib.resources.path('gz_biped_gym.urdf', 'blackbird.sdf') as sdf_path:
        #     absolute_sdf_path = str(sdf_path.resolve())  # Ensure it's an absolute path

        # # Get the full path to `empty.world` in `gz_biped_gym.world`
        # with importlib.resources.path('gz_biped_gym.world', 'empty.world') as temp_world_path:
        #     self.world_path = temp_world_path

        # package_dir = importlib.resources.files('gz_biped_gym')
        
        # os.environ["GZ_SIM_RESOURCE_PATH"] = package_dir.absolute().as_posix()

        # print("package_dir: ", package_dir.absolute().as_posix())
        # print("current dir: ", os.listdir())



        self.sim = blackbird_rl.TrainSimulator(set_gui, world_path)
        obs_low = np.full(32, -np.inf)  # -inf for each element
        obs_high = np.full(32, np.inf)  # inf for each element

        self.observation_space = spaces.Box(low=obs_low, high=obs_high, dtype=np.float64)

        axn_low = np.full(10, -50.0)
        axn_hi = np.full(10, 50.0)
        self.action_space = spaces.Box(low=axn_low, high=axn_hi, dtype=np.float64)
        self.steps = 0
        
        # hyperparameters
        self.Y_WEIGHT = -500.0 # negative to reward distance travelled
        self.POWER_WEIGHT = 0.05 
        self.Z_WEIGHT = 70.0      
        self.x_vel_weight = 100.0
    
    def det_terminal(self):
        """
        determine terminal
        """
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

        torso_contacted = state[32]
        l_contacted = state[33] # 1.0 if contacted 0.0 otherwise
        r_contacted = state[34] # 1.0 if contacted 0.0 otherwise
        legs_contacted = l_contacted or r_contacted
        
        # KEEP THESE. COMMENT/UNCOMMENT ACCORDINGLY
        # print(f"here's l_contacted: {l_contacted}")
        # print(f"here's r_contacted: {r_contacted}")
        # print()

        pose_y = state[1] # robot faces the -y direction
        pose_z = state[2] # height of the robot's position
        
        power = 0.0
        for i in range(len(action)):
            power += action[i] * state[i+22]

        reward = self.Y_WEIGHT*pose_y - self.POWER_WEIGHT*power + legs_contacted*self.Z_WEIGHT*pose_z

        terminal = self.det_terminal()
        if (terminal):
            reward = 10000.0 # terminal_penalty

        self.steps += 1
        return np.array(state, dtype=np.float32), reward, self.det_terminal(), {}

if __name__ == '__main__':
    env = BlackbirdGazebo(render_mode="human")

    for i in range(10000):
        action = np.random.uniform(low=-50.0, high=50.0, size=(10,))
        # action = np.zeros(10)
        state, reward, terminal, _ = env.step(action)
        if (terminal):
            print(f"reached a terminal at idx {i}. resetting...")
            env.reset()