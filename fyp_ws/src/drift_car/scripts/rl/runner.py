import gym
import gym_drift_car
import tensorflow as tf
import tensorflow.contrib.slim as slim
from network_models import DQN
import rospy
from std_msgs.msg import Int8, Float64, Float64MultiArray
import numpy as np

MODEL_PATH = "models/ExpCar7"

env = gym.make('DriftCarGazeboPartialWithAnglesEnv-v0')
pub = rospy.Publisher('matlab_bridge/state', Float64MultiArray, queue_size=1) 

# Network parameters
tf.reset_default_graph()
hidden_size = 500               # number of units in each Q-network hidden layer
learning_rate = 0        # Q-network learning rate
state_size = env.observation_space.shape[0]
action_size = env.action_space.n
mainQN = DQN(name='main', state_size=state_size, action_size=action_size, hidden_size=hidden_size, learning_rate=learning_rate)
targetQN = DQN(name='target', state_size=state_size, action_size=action_size, hidden_size=hidden_size, learning_rate=learning_rate)

radianMappings = [-0.436, -0.261799, -0.0872665, 0, 0.0872665, 0.261799, 0.436] 

with tf.Session() as sess: 
    saver = tf.train.Saver()
    print('Loading latest saved model...')
    ckpt = tf.train.latest_checkpoint(MODEL_PATH)
    saver.restore(sess, ckpt)

    env.render() 
    env.reset()
    # Take one random step to get the pole and cart moving
    state, reward, done, _ = env.step(env.action_space.sample())


    while True:
        feed = {mainQN.inputs_: [state]}
        Qs = sess.run(mainQN.output, feed_dict=feed)
        action = np.argmax(Qs)

        stateArray = Float64MultiArray()
        stateArray.data = state.tolist()
        stateArray.data.append(400)
        stateArray.data.append(radianMappings[action])
        print(stateArray.data)
        pub.publish(stateArray)

        state, reward, done, _ = env.step(action)
        if done:
            env.reset()
            state, reward, done, _ = env.step(env.action_space.sample())