#!/usr/bin/env python
import rospy
import gym
import gym_drift_car
from std_msgs.msg import Int8, Float64, Float64MultiArray
import numpy 
import os
import subprocess
import time
import math

from xbee.thread import XBee
import serial

def callback(data, args):
    action = data.data[0]
    takenOn = data.data[1]
    rospy.loginfo(rospy.get_caller_id() + ' Action: %s', data.data)
    
    env = args[0]
    pub = args[1]
    
    if takenOn == 0: # Action to be taken on the Gazebo Sim
        if(action == -1000):
            rospy.loginfo('Resetting Env . . . \n\n')
            env.reset()
            return
    
        state, reward, done, _ = env.step(action)
        stateArray = Float64MultiArray()
        stateArray.data = state.tolist()
        pub.publish(stateArray)
    else: # Action to be taken on the actual car
        if(action == -1000):
            rospy.loginfo('Resetting Env . . . \n\n')
            #TODO figure out reset of car
            return
        
        action = math.degrees(action)

if __name__ == '__main__':
    # env = gym.make('DriftCarGazeboContinuous-v0')
    env = gym.make('DriftCarGazeboContinuousPartial-v0')

    pub = rospy.Publisher('matlab_bridge/state', Float64MultiArray, queue_size=1) 
    rospy.Subscriber('matlab_bridge/action', Float64MultiArray, callback, (env, pub))

    env.reset()
    env.render()
    
    rospy.spin()