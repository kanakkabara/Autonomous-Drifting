import gym
import gym_drift_car
import tensorflow as tf
import tensorflow.contrib.slim as slim
from network_models import DQN
import rospy
from std_msgs.msg import Int8, Float64, Float64MultiArray
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import style

MODEL_PATH = "models/DQN-Result (4WD, 0.5fr, 1770thr, 45degs)"

env = gym.make('DriftCarGazeboPartialBodyFrame-v0')
pub = rospy.Publisher('drift_car/state', Float64MultiArray, queue_size=1) 

# Network parameters
tf.reset_default_graph()
hidden_size = 500               # number of units in each Q-network hidden layer
learning_rate = 0        # Q-network learning rate
state_size = env.observation_space.shape[0]
action_size = env.action_space.n
mainQN = DQN(name='main', state_size=state_size, action_size=action_size, hidden_size=hidden_size, learning_rate=learning_rate)
targetQN = DQN(name='target', state_size=state_size, action_size=action_size, hidden_size=hidden_size, learning_rate=learning_rate)

fig = plt.figure(figsize=(10, 10))
ax1 = fig.add_subplot(211)
ax1.set_title('DQN Cost')
ax1.set_xlabel('Number of steps')
ax1.set_ylabel('Cost')
plt.ion()
fig.show()
fig.canvas.draw()

with tf.Session() as sess: 
    saver = tf.train.Saver()
    print('Loading latest saved model...')
    ckpt = tf.train.latest_checkpoint(MODEL_PATH)
    saver.restore(sess, ckpt)

    env.render() 
    env.reset()
    state, reward, done, _ = env.step(env.action_space.sample())

    runningReward = []
    while True:
        feed = {mainQN.inputs_: [state]}
        Qs = sess.run(mainQN.output, feed_dict=feed)
        action = np.argmax(Qs)

        state, reward, done, _ = env.step(action)
        runningReward.append(reward)
        ax1.clear()
        ax1.set_title('DQN Cost')
        ax1.set_xlabel('Number of steps')
        ax1.set_ylabel('Cost')
        ax1.plot(runningReward)
        fig.canvas.draw()
        
        if done or len(runningReward) == 150:
            env.reset()
            state, reward, done, _ = env.step(env.action_space.sample())
            runningReward = []
            steps = 0