from model import Actor, Critic
import torch
import numpy as np

# Hyperparameters
GAMMA = 0.99
LEARNING_RATE = 0.001 # for both actor and critic
TAU = 0.001 # soft update of target networks

STATE_SIZE = 32
AXN_SIZE = 10

class BlackBirdDDPG:
    def __init__(self, env, state_size, action_size,):
        
        self.replay_buffer = utils.memory.ReplayMemeory
        self.action_lim = 50.0

        # for SARS vector
        self.state = torch.zeroes(STATE_SIZE) # get initial states from self.reset(state)
        self.action = torch.zeroes(AXN_SIZE)
        self.training = True

        if torch.cuda.is_available():
            self.cuda()

    def eval(self):
        self.actor.eval()
        self.actor_tgt.eval()

        self.critic.eval()
        self.critic_tgt.eval()

    def add_experience(self, r_t, s_t2, terminated):
        """
        AKA observe()
        add experience to replay buffer
        """
        pass

    def random_action(self):
        lo, hi = -self.action_lim, self.action_lim
        action = (hi-lo)*torch.rand(10) + -50.0
        return action

    def reset(self, state):
        pass


    def cuda(self):
        pass

    def seed(self, s):
        pass


    
