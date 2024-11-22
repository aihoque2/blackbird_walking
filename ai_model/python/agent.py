from python.model import Actor, Critic
import torch
import torch.nn.functional as F
import torch.optim as optim

import numpy as np
import python.utils as utils
from python.utils.helper_funcs import *
from python.utils.memory import ReplayMemory
from python.utils.noise_model import OrnsteinUhlenbeckProcess

# Hyperparameters
GAMMA = 0.99
LEARNING_RATE = 0.001 # for both actor and critic
TAU = 0.001 # soft update of target networks

STATE_SIZE = 35
AXN_SIZE = 10

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

class BlackbirdDDPG:
    def __init__(self, env, state_size, action_size):
        
        self.state_size = state_size
        self.action_size = action_size

        self.memory = ReplayMemory(size=int(7e6))
        self.action_lim = 50.0
        self.batch_size = 64

        # our models
        self.actor = Actor(self.state_size, self.action_size, self.action_lim)
        self.actor_tgt = Actor(self.state_size, self.action_size, self.action_lim)
        self.actor_optim = optim.Adam(self.actor.parameters(), lr=LEARNING_RATE)

        self.critic = Critic(self.state_size, self.action_size)
        self.critic_tgt = Critic(self.state_size, self.action_size) # used to calculate y_i
        self.critic_optim  = optim.Adam(self.critic.parameters(), lr=LEARNING_RATE)

        hard_update(self.actor_tgt, self.actor)
        hard_update(self.critic_tgt, self.critic)

        # for SARS vector
        self.s_t = torch.zeros(STATE_SIZE).unsqueeze(0) # get initial states from self.reset(state)
        self.a_t = torch.zeros(AXN_SIZE).unsqueeze(0)
        self.training = True

        self.noise_model = OrnsteinUhlenbeckProcess(theta=0.15, sigma=0.2, mu=0.0, size=self.action_size, )

        # constants
        self.epsilon = 1.0
        self.depsilon = 1.0/50000.0

        if torch.cuda.is_available():
            self.cuda()

    def optimize(self):
        # run the optimization step

        # get SARS vector
        s1, a1, r1, s2, terminal = self.memory.sample(self.batch_size)
        a2 = self.actor_tgt.forward(s2)
        
        term_var = not terminal
        y_i = r1 + GAMMA*term_var*torch.squeeze(self.critic_tgt.forward(s2, a2))
        y_predicted = torch.squeeze(self.critic.forward(s1, a1.detach()))
        
        # Update q function
        loss_critic = F.smooth_l1_loss(y_predicted, y_i) 
        self.critic.optim.zero_grad()
        loss_critic.backward()
        self.critic.optim.step()

        # update pi function
        loss_actor = -1*torch.sum(self.critic.forward(s1, self.actor.forward(s1))) #we wanna max this value, so we trick the optimizer by calculating -
        self.actor.optim.zero_grad()
        loss_actor.backward()
        self.actor.optim.step()

        soft_update(self.critic_tgt, self.critic, TAU)
        soft_update(self.actor_tgt, self.actor, TAU)

    def eval(self):
        self.actor.eval()
        self.actor_tgt.eval()

        self.critic.eval()
        self.critic_tgt.eval()

    def add_experience(self, r_t, s_t2, terminal):
        """
        AKA observe()
        add experience to replay buffer
        """
        if self.training:
            s_t2 = torch.tensor(s_t2, dtype=torch.float32, device=device).unsqueeze(0)
            print("here's s_t2: ", s_t2)
            r_t = torch.tensor([r_t], dtype=torch.float32, device=device)
            terminated = torch.tensor([1.0 if terminal else 0.0], dtype=torch.float32, device=device)

            print(f"add_experience() shape of a_t: {self.a_t.shape}")

            self.memory.append(self.s_t, self.a_t, r_t, s_t2, terminated)
            self.s_t = s_t2
        else:
            raise RuntimeError("add_experience() can only be done in training mode")

    def random_action(self):
        lo, hi = -self.action_lim, self.action_lim
        action = (hi-lo)*torch.rand(10) + -50.0
        self.a_t = action.unsqueeze(0)
        print(f"random_action() shape of a_t:{self.a_t.shape}" )
        return action.numpy()
    
    def select_action(self, s_t, decay_epsilon=True):
        """
        NOTE: this function takes parameter s_t of type torch.Tensor
        """
        action = self.actor(s_t)
        self.a_t = action #save this one for experience replay

        action = action.detach().cpu().numpy()

        #add the noise component to this action
        action += self.training*max(0, self.epsilon)*self.noise_model.sample() 

        if decay_epsilon:
            self.epsilon -= self.depsilon

        print(f"select_action() shape of a_t: {self.a_t.shape}")
        assert torch.is_tensor(self.a_t), "agent's a_t is not set as torch.Tensor"
        return action.reshape(-1)

    def reset(self, state):
         obs = torch.tensor(state, dtype=torch.float32, device=device).unsqueeze(0)
         self.s_t = obs
         self.noise_model.reset_states()

    def cuda(self):
        self.actor.cuda()
        self.actor_tgt.cuda()
        self.critic.cuda()
        self.critic_tgt.cuda()

    def seed(self, s):
        torch.manual_seed(s)
        if (torch.cuda.is_available()):
            torch.cuda.manual_seed(s)

    def save_model(self):
        """
        save the model
        """
        actor_script = torch.jit.script(self.actor)
        actor_script.save('actor_script.pt')


    
