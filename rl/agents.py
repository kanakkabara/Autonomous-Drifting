import random
import numpy as np
import tensorflow as tf
from utilities import target_network_update_apply, target_network_update_ops, ExperienceReplayBuffer
from network_models import DQNetwork


# Agent with double dueling Q network.
class DDQNAgent():
    def __init__(self, sess, config):
        self.primary_Q_network = DQNetwork(
            config.learning_rate,
            config.s_size,
            config.a_size,
            config.h_size,
            config.o_size,
            "primary")
        self.target_Q_network = DQNetwork(
            config.learning_rate,
            config.s_size,
            config.a_size,
            config.h_size,
            config.o_size,
            "target")

        self.saver = tf.train.Saver(save_relative_paths=True)
        tvars = tf.trainable_variables()
        self.target_network_update_operations = target_network_update_ops(
            tvars, config.tau)
        self.experience_buffer = ExperienceReplayBuffer()
        self.batch_size = config.batch_size
        self.gamma = config.gamma
        self.sess = sess

    # Load saved model params.
    def load_agent_state(self, path):
        ckpt = tf.train.get_checkpoint_state(path)
        self.saver.restore(self.sess, ckpt.model_checkpoint_path)

    # Save model params.
    def save_agent_state(self, filename, global_step):
        self.saver.save(sess, filename, global_step=global_step)



    # Add experiences from list to buffer.
    def add_experiences(self, experiences):
        # Experience: [s, action, reward, next_state, d_int]
        self.experience_buffer.add(experiences)



    # Return action that maximizes rewards.
    def take_action(self, state):
        return self.sess.run(self.primary_Q_network.action_predicted, feed_dict={
            self.primary_Q_network.state_input: [state]})[0]


    # Update network params. Returns losses.
    def update_agent(self):
        train_batch = self.experience_buffer.sample_batch(
            self.batch_size)
        # Double DQN
        max_next_state_action = self.sess.run(self.primary_Q_network.action_predicted, feed_dict={
            self.primary_Q_network.state_input: np.vstack(train_batch[:, 3])})
        target_network_Q_values = self.sess.run(self.target_Q_network.Qout, feed_dict={
            self.target_Q_network.state_input: np.vstack(train_batch[:, 3])})

        Q_values_next_state = target_network_Q_values[range(
            self.batch_size), max_next_state_action]
        end_multiplier = (1 - train_batch[:, 4])
        target_Q_values = train_batch[:, 2] + \
            (self.gamma * Q_values_next_state * end_multiplier)

        # Update the primary network.
        l, _ = self.sess.run([self.primary_Q_network.prediction_loss, self.primary_Q_network.update], feed_dict={
            self.primary_Q_network.state_input: np.vstack(train_batch[:, 0]),
            self.primary_Q_network.actions_taken: train_batch[:, 1],
            self.primary_Q_network.target_Q: target_Q_values
        })

        # Update the target network.
        target_network_update_apply(
            self.sess, self.target_network_update_operations)

        return l
