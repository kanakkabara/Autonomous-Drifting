#!/usr/bin/env python
import tensorflow as tf 
import tensorflow.contrib.slim as slim

class DRQN:
  def __init__(self, state_size, action_size, learning_rate=0.01,
    hidden_size=10, rnn_cell=None, name='QNetwork'):
    # state inputs to the Q-network
    with tf.variable_scope(name):
      with tf.name_scope("Prediction"):       
        self.inputs_ = tf.placeholder(dtype=tf.float32, shape=[None, state_size], name='inputs')
        # Resize inputs so that can be processed by Conv2D layers
        pixels = int((state_size//3) ** 0.5)        
        reshaped_inputs = tf.reshape(self.inputs_, shape=[-1, pixels, pixels, 3])

        # Pass through convolutional layers        
        conv = slim.conv2d(inputs=reshaped_inputs, num_outputs=32, kernel_size=[8,8], stride=[4,4], padding='VALID', biases_initializer=None)
        conv = slim.conv2d(inputs=conv, num_outputs=64, kernel_size=[4,4], stride=[2,2], padding='VALID', biases_initializer=None)
        conv = slim.conv2d(inputs=conv, num_outputs=64, kernel_size=[3,3], stride=[1,1], padding='VALID', biases_initializer=None)
        self.conv = slim.conv2d(inputs=conv, num_outputs=hidden_size, kernel_size=[7,7], stride=[1,1], padding='VALID', biases_initializer=None)

        self.train_length = tf.placeholder(dtype=tf.int32)
        self.batch_size = tf.placeholder(dtype=tf.int32, shape=[])
        conv_flat = tf.reshape(slim.flatten(self.conv), [self.batch_size, self.train_length, hidden_size])
        self.state_in = rnn_cell.zero_state(self.batch_size, tf.float32)
        self.rnn, self.rnn_state = tf.nn.dynamic_rnn(inputs=conv_flat, cell=rnn_cell, dtype=tf.float32, initial_state=self.state_in, scope='rnn')
        self.rnn = tf.reshape(self.rnn, shape=[-1,hidden_size])

        # Split into advantage-value streams
        advantage_stream_hid, value_stream_hid = tf.split(self.rnn, 2, 1)
        a_weights = tf.Variable(tf.random_normal([hidden_size//2, action_size]))
        v_weights = tf.Variable(tf.random_normal([hidden_size//2, 1]))
        advantage_stream = tf.matmul(advantage_stream_hid, a_weights)
        value_stream = tf.matmul(value_stream_hid, v_weights)

        # Linear output layer
        self.output = value_stream + tf.subtract(advantage_stream, tf.reduce_mean(advantage_stream, axis=1, keep_dims=True))    

        self.salience = tf.gradients(advantage_stream, reshaped_inputs)

      with tf.name_scope('Training'):
        # One hot encode the actions to later choose the Q-value for the action
        self.actions_ = tf.placeholder(tf.int32, [None], name='actions')
        one_hot_actions = tf.one_hot(self.actions_, action_size)
        # Target Q values for training
        self.targetQs_ = tf.placeholder(tf.float32, [None], name='target')

        ### Train with loss (targetQ - Q)^2
        self.Q = tf.reduce_sum(tf.multiply(self.output, one_hot_actions), axis=1)
        self.td_error = tf.square(self.targetQs_ - self.Q)

        self.maskA = tf.zeros([self.batch_size, self.train_length//2])
        self.maskB = tf.ones([self.batch_size, self.train_length//2])
        self.mask = tf.concat([self.maskA, self.maskB], 1)
        self.mask = tf.reshape(self.mask, [-1])
        self.loss = tf.reduce_mean(self.td_error * self.mask)
        self.opt = tf.train.AdamOptimizer(learning_rate).minimize(self.loss)