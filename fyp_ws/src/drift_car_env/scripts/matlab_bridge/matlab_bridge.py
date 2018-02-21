#!/usr/bin/env python
import rospy
import gym
import gym_drift_car
from std_msgs.msg import Int8, Float64, Float64MultiArray
import numpy 
import os
import subprocess
import time

def callback(data, args):
    rospy.loginfo(rospy.get_caller_id() + ' Action: %s', data.data)
    env = args[0]
    pub = args[1]

    if(data.data == -1000):
        rospy.loginfo('Resetting Env . . . ')
        env.reset()
        return
    
    state, reward, done, _ = env.step(data.data)
    stateArray = Float64MultiArray()
    stateArray.data = state.tolist()
    pub.publish(stateArray)

    # if done:
    #     env.reset()

if __name__ == '__main__':
    #rospy.init_node('matlab_bridge', anonymous=True)
    tmp = os.popen("ps -Af").read()
    roscore_count = tmp.count('roscore')
    if roscore_count == 0:
        subprocess.Popen("roscore")
        time.sleep(1)
        print ("Roscore launched!")

    env = gym.make('DriftCarGazeboContinuous-v0')
    pub = rospy.Publisher('matlab_bridge/state', Float64MultiArray, queue_size=1)    
    rospy.Subscriber('matlab_bridge/action', Float64, callback, (env, pub))
    
    env.reset()
    rospy.spin()