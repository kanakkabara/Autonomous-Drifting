#!/usr/bin/env python
import rospy
import gym
import gym_drift_car
from std_msgs.msg import Float64MultiArray
import math
import numpy as np

from xbee.thread import XBee
import serial

def callback(data, args):
    servo = data.data[0]
    throttle = data.data[1]
    rospy.loginfo(rospy.get_caller_id() + ' Action: %s', data.data)
    
    env = args[0]
    pub = args[1]
    allRewards = args[2]
    
    if(servo == -1000):
        calc(allRewards)
        allRewards = [] 
        rospy.loginfo('Resetting Env . . . \n\n')
        env.reset()       
        return

    state, reward, done, _ = env.step((throttle, servo))
    stateArray = Float64MultiArray()
    stateArray.data = state[-4:-2].tolist()
    # stateArray.data = [state[8], state[10], state[11]]
    pub.publish(stateArray)
    allRewards.append(reward)

def calc(rewards):
    r = np.array(rewards)
    print("Mean: ")
    print(np.mean(r))
    print("Std dev: ")
    print(np.std(r))

if __name__ == '__main__':
    # Default state: x, y, i, j, k, w, xdot, ydot, thetadot, s, xdotbodyframe, ydotbodyframe. 
    env = gym.make('DriftCarGazeboContinuous4WD-v0')
    # env = gym.make('DriftCarGazeboContinuousPartial-v0')
    allRewards = []

    pub = rospy.Publisher('drift_car/state', Float64MultiArray, queue_size=1) 
    rospy.Subscriber('drift_car/action', Float64MultiArray, callback, (env, pub, allRewards))

    env.reset()
    #env.render()
    
    rospy.spin()