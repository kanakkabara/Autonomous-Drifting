#!/usr/bin/env python
import gym
import gym_drift_car
import tensorflow as tf
import numpy as np
from utils import ExperienceReplayBuffer
from network_models import DRQN
import json
import argparse
import datetime
import os
import subprocess
import time

def train(config):
    # Gym environment related variables
    env = gym.make('CollisionGazebo-v0')
    action_size = env.action_space.n
    state_size = env.observation_space.shape[0]

    # Network parameters
    hidden_size = config.h_size                     # number of units in each Q-network hidden layer
    batch_size = config.batch_size                  # experience mini-batch size
    trace_length = config.trace_length

    replay_buffer = ExperienceReplayBuffer(batch_size, trace_length)    

    tf.reset_default_graph()        
    cell = tf.contrib.rnn.BasicLSTMCell(num_units=hidden_size, state_is_tuple=True)
    mainQN = DRQN(name="main", state_size=state_size, action_size=action_size, hidden_size=hidden_size, learning_rate=config.learning_rate, rnn_cell=cell)
    targetCell = tf.contrib.rnn.BasicLSTMCell(num_units=hidden_size, state_is_tuple=True)
    targetQN = DRQN(name="target", state_size=state_size, action_size=action_size, hidden_size=hidden_size, learning_rate=config.learning_rate, rnn_cell=targetCell)
    
    trainable_vars = tf.trainable_variables()
    target_trainables = len(trainable_vars) // 2
    targetQN_vars_update = []
    for ix, var in enumerate(trainable_vars[0:target_trainables]):
        updated_value =  trainable_vars[ix + target_trainables].value() * config.tau + var.value() * (1 - config.tau)
        targetQN_vars_update.append(trainable_vars[ix + target_trainables].assign(updated_value))

    with tf.Session() as sess:
        sess.run(tf.global_variables_initializer())

        saver = tf.train.Saver()
        summary_writer = tf.summary.FileWriter(config.summary_path, sess.graph)
        all_summaries = tf.summary.merge_all()
        
        all_rewards = []
        total_step_count = 0

        if config.load_model:
            print('Loading latest saved model...')
            ckpt = tf.train.latest_checkpoint(config.model_path)
            saver.restore(sess, ckpt)

            for ep in range(40):
                state = env.reset()
                reccurent_state_in = (np.zeros([1, hidden_size]), np.zeros([1, hidden_size]))
                done = False
                episode_reward = 0
                episode_steps = 0
                while not done:
                    Qs = mainQN.get_Q_values(sess, [state/255.0], 1, reccurent_state_in, 1)
                    next_reccurent_state_in = mainQN.rnn_hidden_state(sess, [state/255.0], 1, reccurent_state_in, 1)
                    action = np.argmax(Qs)
                    next_state, reward, done, _ = env.step(action)
                    replay_buffer.add_step([state, action, reward, next_state, done])
                    state = next_state
                    reccurent_state_in = next_reccurent_state_in
                    total_step_count += 1        
                    episode_steps += 1
                    episode_reward += reward
                if len(replay_buffer.episode_buffer) >= trace_length:
                    replay_buffer.flush()
                all_rewards.append(episode_reward)
            total_step_count = 78483
            explore_start = 1.0                             # exploration probability at start
            explore_stop = 0.01                             # minimum exploration probability 
            explore_p = 0.4603
        else:
            # Exploration parameters
            explore_start = 1.0                             # exploration probability at start
            explore_stop = 0.01                             # minimum exploration probability 
            explore_p = 1.0

        for ep in range(2061, config.total_episodes):
            state = env.reset()
            reccurent_state_in = (np.zeros([1, hidden_size]), np.zeros([1, hidden_size]))

            episode_reward = 0
            episode_steps = 0
            while episode_steps < config.max_episode_length:
                if explore_p > np.random.rand() or total_step_count < config.pretrain_steps:
                    # Explore
                    next_reccurent_state_in = mainQN.rnn_hidden_state(sess, [state/255.0], 1, reccurent_state_in, 1)
                    action = env.action_space.sample()
                else:
                    # Exploit
                    Qs = mainQN.get_Q_values(sess, [state/255.0], 1, reccurent_state_in, 1)
                    next_reccurent_state_in = mainQN.rnn_hidden_state(sess, [state/255.0], 1, reccurent_state_in, 1)
                    action = np.argmax(Qs)
                
                # Take action, get new state and reward
                next_state, reward, done, _ = env.step(action)
                replay_buffer.add_step([state, action, reward, next_state, done])

                if total_step_count > config.pretrain_steps:
                    explore_p = explore_stop + (explore_start - explore_stop)*np.exp(-config.epsilon_decay_rate*(total_step_count)) if explore_p > explore_stop else explore_p

                    states, actions, rewards, next_states, dones = replay_buffer.sample() 
                    reccurent_hidden_state_reset = (np.zeros([batch_size, hidden_size]), np.zeros([batch_size, hidden_size])) 

                    for op in targetQN_vars_update:
                        sess.run(op)

                    Q_main_next_state = mainQN.get_Q_values(sess, next_states, trace_length, reccurent_hidden_state_reset, batch_size)
                    action_next_state = np.argmax(Q_main_next_state, axis=1)
                    Q_target_next_state = targetQN.get_Q_values(sess, next_states, trace_length, reccurent_hidden_state_reset, batch_size)
                    target_Qs = Q_target_next_state[range(batch_size * trace_length), action_next_state]

                    targets = (target_Qs * (-(dones - 1)) * config.gamma) + rewards
                    
                    loss = mainQN.update_network(sess, states, targets, actions, trace_length, reccurent_hidden_state_reset, batch_size)
                
                    if total_step_count % config.summary_out_every == 0:
                        scalar_summ = tf.Summary()
                        scalar_summ.value.add(simple_value=explore_p, tag='Explore P')
                        scalar_summ.value.add(simple_value=loss, tag='Mean loss')
                        scalar_summ.value.add(simple_value=np.mean(all_rewards[-10:]), tag='Mean reward')
                        summary_writer.add_summary(scalar_summ, total_step_count)
                        summary_writer.flush()

                state = next_state
                reccurent_state_in = next_reccurent_state_in
                total_step_count += 1        
                episode_steps += 1
                episode_reward += reward

                if done:
                    break

            if len(replay_buffer.episode_buffer) >= trace_length:
                replay_buffer.flush()

            all_rewards.append(episode_reward)
            print('Episode: {}'.format(ep+1), 'Total reward: {}'.format(episode_reward), 'Explore P: {:.4f}'.format(explore_p)) 

            # Save model.
            if config.save_model and total_step_count > config.pretrain_steps and ep % config.save_model_interval == 0:
                print('Saving model...')
                saver.save(sess, config.model_path +'/model' + str(ep+1) + '.ckpt', total_step_count)

if __name__ == '__main__':
    parser = argparse.ArgumentParser()

    parser.add_argument('-bs', "--batch_size", help='Size of batches drawn from the experience buffer for training', type=int, default=16)
    parser.add_argument('-tl', "--trace_length", help='Length of each batch from the experience buffer', type=int, default=8)    
    parser.add_argument('-te', '--total_episodes', help='Number of episodes for training', type=int, default=1000000)
    parser.add_argument('-pre', '--pretrain_steps', help='Initial random steps taken before training', type=int, default=1500)
    parser.add_argument('-hs', '--h_size', help='Units of the hidden layer', type=int, default=512)
    parser.add_argument('-mel', '--max_episode_length', help='Maximum length of an episode', type=int, default=300)
    parser.add_argument('-re', '--render_env', help='Boolean flag on whether the graphics of the env should be shown', action='store_true')
    parser.add_argument('-g', '--gamma', help='Gamma used as the discount factor for credit assignment', type=float, default=0.99)
    parser.add_argument('-t', '--tau', help='Rate of convergence of the target network to the main netork', type=float, default=0.98)
    parser.add_argument('-lr', '--learning_rate', help='Learning rate for the Adam Optimizer', type=float, default=1e-5)
    parser.add_argument('-decay', '--epsilon_decay_rate', help='Rate at which the exploration parameter decays', type=float, default=1e-5)
    parser.add_argument('--save_model_interval', help='Rate of model saving', type=int, default=5)
    parser.add_argument('-soe','--summary_out_every', help='Steps after which summary is written to disk', type=int, default=200)
    parser.add_argument('-lm', '--load_model', help='Load model parameters', action='store_true')
    parser.add_argument('-sm', '--save_model', help='Periodically save model parameters', action='store_true')
    parser.add_argument('--model_path', help='Location on disk where the model is saved', default='./models/' + str(datetime.datetime.now()))
    parser.add_argument('--summary_path', help='Location on disk where the summary is saved', default='./summary/' + str(datetime.datetime.now()))

    config = parser.parse_args()
    train(config)