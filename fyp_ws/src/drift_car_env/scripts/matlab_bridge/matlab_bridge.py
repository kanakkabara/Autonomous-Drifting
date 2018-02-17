#!/usr/bin/env python
import rospy
import gym
import gym_drift_car
from std_msgs.msg import Int8, Float64MultiArray, MultiArrayLayout, MultiArrayDimension
import numpy 
import os
import subprocess
import time

def callback(data, args):
    rospy.loginfo(rospy.get_caller_id() + ' Action: %s', data.data)
    env = args[0]
    pub = args[1]
    
    state, reward, done, _ = env.step(data.data)

    stateArray = Float64MultiArray()
    stateArray.data = state.tolist()
    # stateArray.layout = MultiArrayLayout()
    # stateArray.layout.dim = MultiArrayDimension()
    # stateArray.layout.dim[0].label = 'x'
    # stateArray.layout.dim[1].label = 'y'
    # stateArray.layout.dim[2].label = 'theta'
    # stateArray.layout.dim[3].label = 'xDot'
    # stateArray.layout.dim[4].label = 'yDot'
    # stateArray.layout.dim[5].label = 'thetaDot'
    pub.publish(stateArray)

    if done:
        env.reset()

if __name__ == '__main__':
    #rospy.init_node('matlab_bridge', anonymous=True)
    tmp = os.popen("ps -Af").read()
    roscore_count = tmp.count('roscore')
    if roscore_count == 0:
        subprocess.Popen("roscore")
        time.sleep(1)
        print ("Roscore launched!")

    env = gym.make('DriftCarGazeboEnv-v0')
    pub = rospy.Publisher('matlab_bridge/state', Float64MultiArray, queue_size=1)    
    rospy.Subscriber('matlab_bridge/action', Int8, callback, (env, pub))
    
    env.reset()
    rospy.spin()