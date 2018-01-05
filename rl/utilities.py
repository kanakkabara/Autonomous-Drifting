import tensorflow as tf
import numpy as np
import random


# Get the update operations to perform.
def target_network_update_ops(tvars, tau):
    op_holder = []
    num_variables = len(tvars)
    for ix, var in enumerate(tvars[0:num_variables // 2]):
        # The trainable variables for the target network is
        # in the second half of the list of tvars.
        updated_value = tvars[ix + num_variables //
                              2].value() * (1 - tau) + var.value() * tau
        op_holder.append(tvars[ix + num_variables // 2].assign(updated_value))
    return op_holder


# Apply the update operations on the target network.
def target_network_update_apply(sess, ops):
    for op in ops:
        sess.run(op)


# Helper function to compute the Huber loss with delta = 1.
def huber_loss(diff):
    diff = tf.abs(diff)
    return tf.where(tf.less(diff, 1), tf.square(diff), diff - 0.5)


# Experience replay buffer.
class ExperienceReplayBuffer():
    def __init__(self, max_size=50000):
        self.buffer = []
        self.max_size = max_size

    def add(self, data):
        if len(self.buffer) + len(data) > self.max_size:
            # Evict the oldest experiences if not enough space.
            ix = len(self.buffer) + len(data) - self.max_size
            self.buffer[0:ix] = []
        self.buffer.extend(data)

    def sample_batch(self, batch_size):
        return np.reshape(
            np.array(
                random.sample(
                    self.buffer, batch_size)), [
                batch_size, 5])
