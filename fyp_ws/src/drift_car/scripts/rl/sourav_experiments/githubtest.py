import gym
import gym_drift_car
import tensorflow as tf
import numpy as np
from utils import target_network_update_ops, target_network_update_apply, ExperienceReplayBuffer
from network_models import DQN
import argparse
import datetime

def train(config, env):
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

                target_network_update_apply(sess, targetQN_update)

            # Save model.
            if config.save_model and total_step_count > config.pretrain_steps and \
                    ep % config.save_model_interval == 0:
                print('Saving model...')
                saver.save(sess, config.model_path +'/model' + str(ep) + '.ckpt', total_step_count)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    # Integer hyperparams.
    parser.add_argument(
        '-bs',
        "--batch_size",
        help='The batch size for training',
        type=int,
        default=64)
    parser.add_argument(
        '-te',
        '--total_episodes',
        help='Total number of episodes to run algorithm (Default: 1m)',
        type=int,
        default=10000)
    parser.add_argument(
        '--pretrain_steps',
        help='Number of steps to run algorithm without updating networks',
        type=int,
        default=10000)
    parser.add_argument(
        '--max_episode_length',
        help='Length of each episode',
        type=int,
        default=300)
    parser.add_argument(
        '-re',
        '--render_env',
        help='Render learning environment',
        action='store_true')

    # Float hyperparameters.
    parser.add_argument(
        '-g',
        '--gamma',
        help='Discount factor',
        type=float,
        default=0.99)
    parser.add_argument(
        '--tau',
        help='Controls update rate of target network',                        
        type=float,
        default=0.999)
    parser.add_argument(
        '-lr',
        '--learning_rate',
        help='Learning rate of algorithm',
        type=float,
        default=1e-4)
        #default=1e-10)
    parser.add_argument(
        '-edr',
        '--epsilon_decay_rate',
        help='Rate of epsilon decay',
        type=float,
        default=0.0001)
        #default=1e-5)


    # Intervals.
    parser.add_argument(
        '--save_model_interval',
        help='How often to save model. (Default: 5 ep)',
        type=int,
        default=5)
    parser.add_argument(
        '-eui',
        '--epsilon_update_interval',
        help='How often to update epsilon (Default: 10k steps)',
        type=int,
        default=1e5)
    parser.add_argument(
        '-soe',
        '--summary_out_every',
        help='How often to print out summaries (Default: 200 steps)',
        type=int,
        default=200)
    parser.add_argument(
        '--chart_refresh_interval',
        help='Number of episodes between chart updates (Default: 100 ep)',
        type=int,
        default=100)

    # Model load/save and path.
    parser.add_argument(
        '-lm',
        '--load_model',
        help='Load saved model parameters',
        action='store_true')
    parser.add_argument(
        '-sm',
        '--save_model',
        help='Periodically save model parameters',
        action='store_true')
    parser.add_argument(
        '--model_path',
        help='Path of saved model parameters',                        
        default='./models/'+str(datetime.datetime.now()))
    parser.add_argument(
        '--summary_path',
        help='Path of training summary',
        default='./summary/'+str(datetime.datetime.now()))
    parser.add_argument(
        '--verbose',
        help='Verbose output',
        action='store_true')
    
    config = parser.parse_args()

    # env = gym.make('CartPole-v0')
    env = gym.make('DriftCarGazeboEnv-v0')
    # env = gym.make('MountainCar-v0')
    
    # Additional network params.
    vars(config)['h_size'] = 500
    # Train the network.
    train(config, env)