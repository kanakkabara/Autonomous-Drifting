#!/usr/bin/env python
import numpy as np
import random
from collections import deque

# Experience replay buffer.
class ExperienceReplayBuffer():
    def __init__(self, batch_size, trace_length, max_size=1e6):
        self.buffer = deque(maxlen=int(max_size))
        self.episode_buffer = []
        self.batch_size = batch_size
        self.trace_length = trace_length

    def add_step(self, step):
        self.episode_buffer.append(np.reshape(np.array(step), [1,5]))

    def flush(self):
        self.buffer.append(list(zip(np.array(self.episode_buffer))))
        self.episode_buffer = []
            
    def sample(self):
        sampled_traces = []
        for episode in random.sample(self.buffer, self.batch_size):
            point = np.random.randint(0, len(episode) + 1 - self.trace_length)
            sampled_traces.append(episode[point: point + self.trace_length])
        
        batch = np.reshape(np.array(sampled_traces), [self.batch_size * self.trace_length, 5])
        states = np.vstack(batch[:, 0]/255.0)
        actions = batch[:, 1]
        rewards = batch[:, 2]
        next_states = np.vstack(batch[:, 3]/255.0)
        dones = batch[:, 4]

        return states, actions, rewards, next_states, dones