#!/usr/bin/env python
import tensorflow as tf 

class DQN:
    def __init__(self, state_size, action_size, learning_rate=0.01, 
                 hidden_size=10, 
                 name='QNetwork'):
        # state inputs to the Q-network
        with tf.variable_scope(name):
            with tf.name_scope("Prediction"):
                self.inputs_ = tf.placeholder(tf.float32, [None, state_size], name='inputs')
                # ReLU hidden layers
                self.fc1 = self._linear(self.inputs_, hidden_size, scope='layer1')
                self.fc2 = self._linear(self.fc1, hidden_size, scope='layer2')
                self.fc3 = self._linear(self.fc2, hidden_size, scope='layer3')

                value_stream_hid = self._linear(self.fc3, hidden_size//2, scope='value_hid')
                #tf.contrib.layers.fully_connected(self.fc3, hidden_size//2)
                value_stream = self._linear(value_stream_hid, 1, activation_fn=None, scope='value_stream')
                #tf.contrib.layers.fully_connected(value_stream_hid, 1, activation_fn=None)

                advantage_stream_hid = self._linear(self.fc3, hidden_size//2, scope='adv_hid')
                #tf.contrib.layers.fully_connected(self.fc3, hidden_size//2)
                advantage_stream = self._linear(advantage_stream_hid, action_size, activation_fn=None, scope='adv_stream')
                #tf.contrib.layers.fully_connected(advantage_stream_hid, action_size, activation_fn=None)
                

                # Linear output layer
                self.output = value_stream + tf.subtract(advantage_stream, tf.reduce_mean(advantage_stream, axis=1, keep_dims=True))    

            with tf.name_scope('Training'):
                # One hot encode the actions to later choose the Q-value for the action
                self.actions_ = tf.placeholder(tf.int32, [None], name='actions')
                one_hot_actions = tf.one_hot(self.actions_, action_size)
                # Target Q values for training
                self.targetQs_ = tf.placeholder(tf.float32, [None], name='target')

                ### Train with loss (targetQ - Q)^2
                self.Q = tf.reduce_sum(tf.multiply(self.output, one_hot_actions), axis=1)
                
                self.loss = tf.reduce_mean(tf.square(self.targetQs_ - self.Q))
                self.opt = tf.train.AdamOptimizer(learning_rate).minimize(self.loss)

    def _linear(self, input, o_size, activation_fn=tf.nn.relu, scope="linear"):
        input_size = input.get_shape().as_list()[1]
        with tf.variable_scope(scope):
            W = tf.get_variable('Weights', shape=[input_size, o_size], initializer=tf.contrib.layers.xavier_initializer())
            variable_summaries(W)
            b = tf.get_variable('b', shape=[o_size], initializer=tf.constant_initializer(0.02))
            variable_summaries(b)
            
            out = tf.nn.bias_add(tf.matmul(input, W), b)
            if activation_fn:
                out = activation_fn(out)
            tf.summary.histogram('activations', out)
            return out

def variable_summaries(var):
  """Attach a lot of summaries to a Tensor (for TensorBoard visualization)."""
  mean = tf.reduce_mean(var)
  tf.summary.scalar('mean', mean)
  with tf.name_scope('stddev'):
    stddev = tf.sqrt(tf.reduce_mean(tf.square(var - mean)))
  tf.summary.scalar('stddev', stddev)
  tf.summary.scalar('max', tf.reduce_max(var))
  tf.summary.scalar('min', tf.reduce_min(var))
  tf.summary.histogram('histogram', var)