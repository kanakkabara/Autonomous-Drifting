#!/usr/bin/env python
import tensorflow as tf
import numpy as np
import random
from collections import deque


# Get the update operations to perform.
def target_network_update_ops(tvars, tau):
    op_holder = []
    num_variables = len(tvars)
    for ix, var in enumerate(tvars[0:num_variables // 2]):
        # The trainable variables for the target network is
        # in the second half of the list of tvars.
        updated_value =  tvars[ix + num_variables // 2].value() * tau + var.value() * (1 - tau)
        op_holder.append(tvars[ix + num_variables // 2].assign(updated_value))
    return op_holder

# Apply the update operations on the target network.
def target_network_update_apply(sess, ops):
    for op in ops:
        sess.run(op)

# Experience replay buffer.
class ExperienceReplayBuffer():
    def __init__(self, max_size=1e6):
        self.buffer = deque(maxlen=int(max_size))
    
    def add(self, experience):
        self.buffer.append(experience)
            
    def sample(self, batch_size, trace_length):
        sampled_traces = []
        for episode in random.sample(self.buffer, batch_size):
            point = np.random.randint(0, len(episode)+1-trace_length)
            sampled_traces.append(episode[point:point+trace_length])
        
        batch = np.reshape(np.array(sampled_traces), [batch_size*trace_length, 5])
        states = np.vstack(batch[:, 0]/255.0)
        actions = batch[:, 1]
        rewards = batch[:, 2]
        next_states = np.vstack(batch[:, 3]/255.0)
        dones = batch[:, 4]

        return states, actions, rewards, next_states, dones