"""
helper_funcs.py

file used to hold helper functions
"""

import torch
import numpy as np

USE_CUDA = torch.cuda.is_available()

#update functions
def soft_update(target, source, tau):
    for target_param, param in zip(target.parameters(), source.parameters()):
        target_param.data.copy_(
            tau*param.data + (1.0 - tau)*target_param.data
        )

def hard_update(target, source):
    for target_param, param in zip(target.parameters(), source.parameters()):
            target_param.data.copy_(param.data)
