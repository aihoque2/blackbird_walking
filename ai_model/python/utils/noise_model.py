#!/usr/local/bin/python3

#noise_model.py
from abc import ABC, abstractclassmethod
import numpy as np

class RandomProcess(ABC):
    @abstractclassmethod
    def reset_states():
        pass


class OrnsteinUhlenbeckProcess(RandomProcess):
    """
    Noise process used for the policy 
    when generating an action when learnring
    """
    def __init__(self, theta, mu=0., sigma=1., dt=1e-2, x0=None, size=1, sigma_min=None, n_steps_annealing=1000):
        self.mu = mu
        self.sigma=sigma
        self.n_steps = 0
        self.theta = theta
        self.dt = dt
        self.x0 = x0
        self.size=size
        self.reset_states()

        if sigma_min is not None:
            self.m = -float(sigma-sigma_min)/float(n_steps_annealing)
            self.c = sigma
            self.sigma_min = self.sigma_min
        else:
            self.m = 0
            self.c = sigma
            self.sigma_min = sigma

    @property
    def current_sigma(self):
        sigma = max(self.sigma_min, self.m*float(self.n_steps)+self.c)
        return sigma

    def sample(self):
        x = self.x_prev + self.theta * (self.mu - self.x_prev) * self.dt + self.current_sigma * np.sqrt(self.dt) * np.random.normal(size=self.size)
        self.x_prev = x
        self.n_steps += 1
        return x

    def reset_states(self):
        self.x_prev = self.x0 if self.x0 is not None else np.zeros(self.size)

    