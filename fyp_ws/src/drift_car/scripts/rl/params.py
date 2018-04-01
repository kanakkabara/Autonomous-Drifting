import argparse
import datetime
import os


def parse_args():
  parser = argparse.ArgumentParser()

  # Defaults
  default_environment = 'CartPole-v0'
  default_learning_rate = 1e-5
  default_batch_size = 64
  default_total_episodes = 1000000
  default_pretrain_steps = 10000
  default_hidden_layer_size = 500
  default_max_episode_length = 300
  default_discount_factor = 0.99
  default_target_network_update_rate = 0.98    # Change this.
  default_epsilon_decay_rate = 1e-5
  default_mode = 'train'
  default_save_model_interval = 5
  default_summary_out_interval = 200
  default_summary_path = './summary/' + str(datetime.datetime.now())
  default_model_path = './models/' + str(datetime.datetime.now())


  parser.add_argument('--env', help='The environment to use. eg: CartPole-v0', default=default_environment)
  parser.add_argument('--mode', help='Mode of the run. eg: train, runpolicy', default=default_mode)

  parser.add_argument(
    '-bs',
    "--batch_size",
    help='The batch size for training',
    type=int,
    default=default_batch_size)
  parser.add_argument('-te', 
    '--total_episodes', 
    help='Total number of episodes to run algorithm (Default: 1m)', 
    type=int,
    default=default_total_episodes)
  parser.add_argument(
    '--pretrain_steps',
    help='Number of steps to run algorithm without updating networks',
    type=int,
    default=default_pretrain_steps)

  parser.add_argument(
    '--h_size', 
    help='Number of units in hidden layer', 
    type=int,
    default=default_hidden_layer_size)

  parser.add_argument(
    '--max_episode_length',
    help='Length of each episode',
    type=int,
    default=default_max_episode_length)
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
    default=default_discount_factor)
  parser.add_argument(
    '--tau',
    help='Controls update rate of target network',                        
    type=float,
    default=default_target_network_update_rate)
  parser.add_argument(
    '-lr',
    '--learning_rate',
    help='Learning rate of algorithm',
    type=float,
    default=default_learning_rate)
  #default=1e-4)
  parser.add_argument(
    '-edr',
    '--epsilon_decay_rate',
    help='Rate of epsilon decay',
    type=float,
    default=default_epsilon_decay_rate)
    #default=0.0001)


  # Intervals.
  parser.add_argument(
    '--save_model_interval',
    help='How often to save model. (Default: 5 ep)',
    type=int,
    default=default_save_model_interval)
  parser.add_argument(
    '-soe',
    '--summary_out_every',
    help='How often to print out summaries (Default: 200 steps)',
    type=int,
    default=default_summary_out_interval)

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
    default=default_model_path)
  parser.add_argument(
    '--summary_path',
    help='Path of training summary',
    default=default_summary_path)
  parser.add_argument(
    '--verbose',
    help='Verbose output',
    action='store_true')

  return parser.parse_args()