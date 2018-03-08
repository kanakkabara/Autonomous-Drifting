#!/usr/bin/env python
import gym
#import gym_drift_car
import tensorflow as tf
import numpy as np
from utils import target_network_update_ops, target_network_update_apply, ExperienceReplayBuffer
from network_models import DQN
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
    hidden_size = config.h_size               # number of units in each Q-network hidden layer
    learning_rate = config.learning_rate         # Q-network learning rate

    # Memory parameters
    batch_size = config.batch_size                # experience mini-batch size
    pretrain_length = batch_size   # number experiences to pretrain the memory


    tf.reset_default_graph()
    action_size = env.action_space.n
    state_size = env.observation_space.shape[0]
    mainQN = DQN(name='main', state_size=state_size, action_size=action_size, hidden_size=hidden_size, learning_rate=learning_rate)
    targetQN = DQN(name='target', state_size=state_size, action_size=action_size, hidden_size=hidden_size, learning_rate=learning_rate)
    targetQN_update = target_network_update_ops(tf.trainable_variables(), tau=config.tau)

    # Initialize the simulation
    env.reset()
    # Take one random step to get the pole and cart moving
    state, reward, done, _ = env.step(env.action_space.sample())

    memory = ExperienceReplayBuffer()

    # Make a bunch of random actions and store the experiences
    for ii in range(pretrain_length):
        # Uncomment the line below to watch the simulation
        # env.render()

        # Make a random action
        action = env.action_space.sample()
        next_state, reward, done, _ = env.step(action)

        if done:
            # The simulation fails so no next state
            next_state = np.zeros(state.shape)
            # Add experience to memory
            memory.add((state, action, reward, next_state))
            
            # Start new episode
            env.reset()
            # Take one random step to get the pole and cart moving
            state, reward, done, _ = env.step(env.action_space.sample())
        else:
            # Add experience to memory
            memory.add((state, action, reward, next_state))
            state = next_state




    # Now train with experiences
    saver = tf.train.Saver()
    rewards_list = []
    #with tf.device('/gpu:0'):
    with tf.Session() as sess:
        # Initialize variables
        sess.run(tf.global_variables_initializer())
        summary_writer = tf.summary.FileWriter(config.summary_path, sess.graph)
        all_summaries = tf.summary.merge_all()
        if config.load_model:
            print('Loading latest saved model...')
            ckpt = tf.train.latest_checkpoint(config.model_path)
            saver.restore(sess, ckpt)
            

        total_step_count = 0
        for ep in range(1, train_episodes):
            total_reward = 0
            t = 0
            while t < max_steps:
                total_step_count += 1
                if config.render_env:
                    env.render() 
                
                # Explore or Exploit
                explore_p = explore_stop + (explore_start - explore_stop)*np.exp(-decay_rate*total_step_count) 
                if config.load_model:
                    explore_p = 0
                if explore_p > np.random.rand():
                    # Make a random action
                    action = env.action_space.sample()
                else:
                    # Get action from Q-network
                    feed = {mainQN.inputs_: [state]}
                    Qs = sess.run(mainQN.output, feed_dict=feed)
                    action = np.argmax(Qs)
                
                # Take action, get new state and reward
                next_state, reward, done, _ = env.step(action)
        
                total_reward += reward
                
                if done:
                    # the episode ends so no next state
                    next_state = np.zeros(state.shape)
                    t = max_steps
                    
                    print('Episode: {}'.format(ep),
                        'Total reward: {}'.format(total_reward),
                        'Training loss: {:.4f}'.format(loss),
                        'Explore P: {:.4f}'.format(explore_p))
                    rewards_list.append(total_reward)
                    
                    # Add experience to memory
                    memory.add((state, action, reward, next_state))
                    
                    # Start new episode
                    env.reset()
                    # Take one random step to get the pole and cart moving
                    state, reward, done, _ = env.step(env.action_space.sample())

                else:
                    # Add experience to memory
                    memory.add([state, action, reward, next_state])
                    state = next_state
                    t += 1
                
                # Sample mini-batch from memory
                for i in range(3):
                    batch = memory.sample(batch_size)
                    states = np.vstack(batch[:, 0])
                    actions = batch[:,1]
                    rewards = batch[:, 2]
                    next_states = np.vstack(batch[:, 3])
                    
                    # Train network
                    Q_main_next_state = sess.run(mainQN.output, feed_dict={mainQN.inputs_:next_states})
                    action_next_state = np.argmax(Q_main_next_state, axis=1)
                    Q_target_next_state = sess.run(targetQN.output, feed_dict={targetQN.inputs_:next_states})
                    target_Qs = Q_target_next_state[range(0, batch_size), action_next_state]
                    
                    # Set target_Qs to 0 for states where episode ends
                    episode_ends = (next_states == np.zeros(states[0].shape)).all(axis=1)
                    target_Qs[episode_ends] = 0
                    
                    targets = rewards + gamma * target_Qs
                
                    loss, _ = sess.run([mainQN.loss, mainQN.opt],
                                        feed_dict={mainQN.inputs_: states,
                                                mainQN.targetQs_: targets,
                                                mainQN.actions_: actions
                                                # # TODO FIX ITTT
                                                # targetQN.inputs_: states
                                                })

                if total_step_count % config.summary_out_every == 0:
                    scalar_summ = tf.Summary()
                    scalar_summ.value.add(simple_value=explore_p, tag='Explore P')
                    scalar_summ.value.add(simple_value=loss, tag='Mean loss')
                    scalar_summ.value.add(simple_value=np.mean(rewards_list[-10:]), tag='Mean reward')
                    #summary_writer.add_summary(summ, total_step_count)
                    summary_writer.add_summary(scalar_summ, total_step_count)
                    summary_writer.flush()

                    if os.path.exists(config.summary_path):
                        with open(config.summary_path+'/hyperparams.json', 'w') as fp:
                            jsonDict = {"config" : vars(config)}
                            jsonDict["total_step_count"] = total_step_count
                            jsonDict["explore_p"] = explore_p
                            json.dump(jsonDict, fp, sort_keys=True, indent=4)

                target_network_update_apply(sess, targetQN_update)

            # Save model.
            if config.save_model and total_step_count > config.pretrain_steps and \
                    ep % config.save_model_interval == 0:
                print('Saving model...')
                saver.save(sess, config.model_path +'/model' + str(ep) + '.ckpt', total_step_count)

if __name__ == '__main__':
    config = params.parse_args()
    train(config)
