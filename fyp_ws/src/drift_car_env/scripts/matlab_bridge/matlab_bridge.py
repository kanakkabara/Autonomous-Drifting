#!/usr/bin/env python
import rospy
import gym
import gym_drift_car
from std_msgs.msg import Float64MultiArray
import math

from xbee.thread import XBee
import serial

def callback(data, args):
    action = data.data[0]
    takenOn = data.data[1]
    rospy.loginfo(rospy.get_caller_id() + ' Action: %s', data.data)
    
    env = args[0]
    pub = args[1]
    
    if(action == -1000):
        rospy.loginfo('Resetting Env . . . \n\n')
        env.reset()
        return

    state, reward, done, _ = env.step(action)
    stateArray = Float64MultiArray()
    stateArray.data = state.tolist()
    pub.publish(stateArray)

if __name__ == '__main__':
    env = gym.make('DriftCarGazeboContinuous4WD-v0')
    # env = gym.make('DriftCarGazeboContinuousPartial-v0')

    pub = rospy.Publisher('drift_car/state', Float64MultiArray, queue_size=1) 
    rospy.Subscriber('drift_car/action', Float64MultiArray, callback, (env, pub))

    env.reset()
    #env.render()
    
    rospy.spin()
