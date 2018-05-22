#!/usr/bin/env python
import gym
import gym_drift_car
import tensorflow as tf
import numpy as np
from utils import target_network_update_ops, target_network_update_apply, ExperienceReplayBuffer
from network_models import DRQN
import json
import os
import params

def train(config):
    # Create environment
    env = gym.make(config.env)

    train_episodes = config.total_episodes          # max number of episodes to learn from
    max_steps = config.max_episode_length           # max steps in an episode
    gamma = config.gamma                            # future reward discount

    # Exploration parameters
    explore_start = 1.0                             # exploration probability at start
    explore_stop = 0.01                             # minimum exploration probability 
    decay_rate = config.epsilon_decay_rate          # exponential decay rate for exploration prob

    # Network parameters
    hidden_size = config.h_size                     # number of units in each Q-network hidden layer
    learning_rate = config.learning_rate            # Q-network learning rate
    trace_length = 8

    # Memory parameters
    batch_size = config.batch_size                  # experience mini-batch size
    pretrain_length = config.pretrain_steps         # number experiences to pretrain the memory

    tf.reset_default_graph()
    action_size = env.action_space.n
    state_size = env.observation_space.shape[0]
    
    cell = tf.contrib.rnn.BasicLSTMCell(num_units=hidden_size, state_is_tuple=True)
    mainQN = DRQN(name='main', state_size=state_size, action_size=action_size, hidden_size=hidden_size, learning_rate=learning_rate, rnn_cell=cell)
    cellT = tf.contrib.rnn.BasicLSTMCell(num_units=hidden_size, state_is_tuple=True)
    targetQN = DRQN(name='target', state_size=state_size, action_size=action_size, hidden_size=hidden_size, learning_rate=learning_rate, rnn_cell=cellT)
    targetQN_update = target_network_update_ops(tf.trainable_variables(), tau=config.tau)
    memory = ExperienceReplayBuffer()

    with tf.Session() as sess:
        # Initialize variables
        sess.run(tf.global_variables_initializer())
        
        saver = tf.train.Saver()
        summary_writer = tf.summary.FileWriter(config.summary_path, sess.graph)
        all_summaries = tf.summary.merge_all()
            
        rewards_list = []
        total_step_count = 0
        explore_p = 1.0

        if config.load_model:
            print('Loading latest saved model...')
            ckpt = tf.train.latest_checkpoint(config.model_path)
            saver.restore(sess, ckpt)

        for ep in range(1, train_episodes):
            memory_buffer = []
            state = env.reset()
            state_in = (np.zeros([1,hidden_size]), np.zeros([1,hidden_size]))

            total_reward = 0
            t = 0
            while t < max_steps:
                # Explore or Exploit
                feed = {mainQN.inputs_: [state/255.0], mainQN.train_length: 1, mainQN.state_in: state_in, mainQN.batch_size: 1}
                if explore_p > np.random.rand() or total_step_count < pretrain_length:
                    # Make a random action
                    next_state_in = sess.run(mainQN.rnn_state, feed_dict=feed)
                    action = env.action_space.sample()
                else:
                    # Get action from Q-network
                    Qs, next_state_in = sess.run([mainQN.output, mainQN.rnn_state], feed_dict=feed)
                    action = np.argmax(Qs)
                
                # Take action, get new state and reward
                next_state, reward, done, _ = env.step(action)
                
                memory_buffer.append(np.reshape(np.array([state, action, reward, next_state, done]), [1,5]))

                if total_step_count > pretrain_length:
                    explore_p = explore_stop + (explore_start - explore_stop)*np.exp(-decay_rate*(total_step_count)) if explore_p > explore_stop else explore_p

                    target_network_update_apply(sess, targetQN_update)
                    #Reset the recurrent layer's hidden state
                    state_train = (np.zeros([batch_size, hidden_size]),np.zeros([batch_size, hidden_size])) 
                    
                    #Get a random batch of experiences.
                    states, actions, rewards, next_states, dones = memory.sample(batch_size, trace_length) 

                    Q_main_next_state = sess.run(mainQN.output, feed_dict={mainQN.inputs_: next_states, mainQN.train_length: trace_length, mainQN.state_in: state_train, mainQN.batch_size: batch_size})
                    action_next_state = np.argmax(Q_main_next_state, axis=1)
                    Q_target_next_state = sess.run(targetQN.output,feed_dict={targetQN.inputs_: next_states, targetQN.train_length: trace_length, targetQN.state_in: state_train, targetQN.batch_size: batch_size})
                    target_Qs = Q_target_next_state[range(batch_size * trace_length), action_next_state]

                    end_multiplier = -(dones - 1)
                    targets = rewards + (gamma * target_Qs * end_multiplier)
                    #Update the network with our target values.
                    loss, _ = sess.run([mainQN.loss, mainQN.opt], \
                        feed_dict={mainQN.inputs_: states, mainQN.targetQs_: targets,\
                        mainQN.actions_: actions, mainQN.train_length: trace_length,\
                        mainQN.state_in: state_train, mainQN.batch_size: batch_size})

                    if total_step_count % config.summary_out_every == 0:
                        scalar_summ = tf.Summary()
                        scalar_summ.value.add(simple_value=explore_p, tag='Explore P')
                        scalar_summ.value.add(simple_value=loss, tag='Mean loss')
                        scalar_summ.value.add(simple_value=np.mean(rewards_list[-10:]), tag='Mean reward')
                        summary_writer.add_summary(scalar_summ, total_step_count)
                        summary_writer.flush()

                state = next_state
                state_in = next_state_in
                total_step_count += 1        
                t += 1
                total_reward += reward

                if done:
                    break

            if len(memory_buffer) >= trace_length:
                memory.add(list(zip(np.array(memory_buffer))))
            rewards_list.append(total_reward)
            print('Episode: {}'.format(ep), 'Total reward: {}'.format(total_reward), 'Explore P: {:.4f}'.format(explore_p)) 

            # Save model.
            if config.save_model and total_step_count > pretrain_length and ep % config.save_model_interval == 0:
                print('Saving model...')
                saver.save(sess, config.model_path +'/model' + str(ep) + '.ckpt', total_step_count)

if __name__ == '__main__':
    config = params.parse_args()
    train(config)