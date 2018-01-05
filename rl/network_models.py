import tensorflow as tf
import tensorflow.contrib.slim as slim
import numpy as np
from utilities import huber_loss

# Dueling DQN model.


class DQNetwork():
    def __init__(self, lr, s_size, a_size, h_size, o_size, name):
        if o_size % 2 != 0:
            raise ValueError('Number of outputs from final layer must be even')

        # Forward pass of the network.
        # output: batch_size x s_size
        self.state_input = tf.placeholder(
            shape=[None, s_size], dtype=tf.float32)

        #initializer = tf.truncated_normal_initializer(0, 0.02)
        # output: batch_size x h_size
        layer1 = self.relu_linear_layer(
            self.state_input, h_size, name + "-layer1")
        # Layer 2.
        layer2 = self.relu_linear_layer(layer1, h_size, name + "-layer2")
        # Layer 3
        layer3 = self.relu_linear_layer(layer2, h_size, name + "-layer3")

        # Layer 4
        layer4 = self.relu_linear_layer(layer3, h_size, name + "-layer4")

        value = self.relu_linear_layer(layer4, o_size, name+'-value1')
        value = slim.fully_connected(value, 1, 
            biases_initializer=tf.constant_initializer(0.02),
            weights_initializer=tf.truncated_normal_initializer(0, 0.02),
            activation_fn=None,
            scope=name+'-value2')

        advantage = self.relu_linear_layer(layer3, o_size, name+'-adv1')
        advantage = slim.fully_connected(advantage, a_size,
            biases_initializer=tf.constant_initializer(0.02),
            weights_initializer=tf.truncated_normal_initializer(0, 0.02),
            activation_fn=None,
            scope=name+'-adv2')

        self.Qout = value + tf.subtract(advantage, tf.reduce_mean(advantage, axis=1, keep_dims=True))
        # Predict action that maximizes Q-value.
        self.action_predicted = tf.argmax(self.Qout, axis=1)

        # Evaluate loss and backward pass.
        self.target_Q = tf.placeholder(shape=[None], dtype=tf.float32, name='target_q')
        self.actions_taken = tf.placeholder(shape=[None], dtype=tf.int32, name='action_taken')
        self.action_taken_one_hot = tf.one_hot(
            self.actions_taken, a_size, 1.0, 0.0, dtype=tf.float32, name='action_one_hot')
        self.predicted_Q = tf.reduce_sum(self.Qout * self.action_taken_one_hot, axis=1)

        self.prediction_loss = tf.reduce_mean(
            huber_loss(self.predicted_Q - self.target_Q))

        # Optimizer
        self.optimizer = tf.train.AdamOptimizer(learning_rate=lr)
        self.update = self.optimizer.minimize(self.prediction_loss)

    def relu_linear_layer(self, x, out_size, scope='relu_batch_norm'):
        initializer = tf.truncated_normal_initializer()
        return slim.fully_connected(x, out_size,
                                    biases_initializer=tf.constant_initializer(0.02),
                                    weights_initializer=initializer,
                                    activation_fn=tf.nn.relu,
                                    scope=scope)
