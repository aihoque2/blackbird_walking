from model import Actor, Critic
import torch
import numpy as np
from utils.helper_funcs import *

# Hyperparameters
GAMMA = 0.99
LEARNING_RATE = 0.001 # for both actor and critic
TAU = 0.001 # soft update of target networks

STATE_SIZE = 32
AXN_SIZE = 10

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

class BlackBirdDDPG:
    def __init__(self, env, state_size, action_size,):
        
        self.memory = utils.memory.ReplayMemeory
        self.action_lim = 50.0
        self.batch_size = 64

        # our models
        self.actor = Actor(self.state_size, self.action_size, action_lim)
        self.actor_tgt = Actor(self.state_size, self.action_size, action_lim)
        self.actor_optim = Adam(self.actor.parameters(), lr=self.prate)

        self.critic = Critic(self.state_size, self.action_size)
        self.critic_tgt = Critic(self.state_size, self.action_size) # used to calculate y_i
        self.critic_optim  = Adam(self.critic.parameters(), lr=self.rate)

        hard_update(self.actor_tgt, self.actor)
        hard_update(self.critic_tgt, self.critic)

        # for SARS vector
        self.s_t = torch.zeroes(STATE_SIZE) # get initial states from self.reset(state)
        self.a_t = torch.zeroes(AXN_SIZE)
        self.training = True

        if torch.cuda.is_available():
            self.cuda()

    def optimize(self):
        # run the optimization step

        # get SARS vector
        s1, a1, r1, s2, terminal = self.memory.sample(self.batch_size)
        a2 = self.actor_tgt.forward(s2)
        
        term_var = not terminal
        y_i = r1 + GAMMA*term_var*torch.squeeze(self.critic_tgt.forward(s2, a2))
        y_predicted = torch.squeeze(self.critic.forward(s1, a1))
        
        # Update q function
    

        # update pi function 
        pass

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
        if self.is_training:
            s_t2 = torch.tensor(s_t2, dtype=torch.float32, device=device).unsqueeze(0)
            r_t = torch.tensor([r_t], dtype=torch.float32, device=device)
            terminated = torch.tensor([1.0 if terminated else 0.0], dtype=torch.float32, device=device)
            self.memory.append(self.s_t, self.a_t, r_t, s_t2, terminated)
            self.s_t = s_t2
        else:
            raise RuntimeError("add_exeperience can only be done in training mode")

    def random_action(self):
        lo, hi = -self.action_lim, self.action_lim
        action = (hi-lo)*torch.rand(10) + -50.0
        self.a_t = action
        return action.numpy()

    def reset(self, state):
         obs = torch.tensor(state, dtype=torch.float32, device=device).unsqueeze(0)
         selt.s_t = obs
         self.noise_model.reset_states()

    def cuda(self):
        self.actor.cuda()
        self.actor_tgt.cuda()
        self.critic.cuda()
        self.critic_tgt.cuda()

    def seed(self, s):
        pass


    
