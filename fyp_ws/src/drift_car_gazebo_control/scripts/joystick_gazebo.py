#!/usr/bin/env python
import rospy
import gym
import gym_drift_car
from sensor_msgs.msg import Joy
import subprocess

global throtle, servo, drifting, env
throtle = 250
driftThrotle = 1720
THROTLE_STEP = 2
servo = 0
drifting = False

env = None

def translate(value, leftMin, leftMax, rightMin, rightMax):
    leftSpan = leftMax - leftMin
    rightSpan = rightMax - rightMin
    valueScaled = float(value - leftMin) / float(leftSpan)
    return rightMin + (valueScaled * rightSpan)

def callback(data):
    if data.buttons[1] == 1: #B Button for Reset
        env.reset()
        return
        
    global throtle, servo, drifting, driftThrotle
    servo = data.axes[3] * 0.46
    throtle = translate(data.axes[2], -1.0, 1.0, driftThrotle, 1650)

    if data.buttons[4] == 1: # LB Button to stop drift throttle
        drifting = False
    if data.buttons[5] == 1: # RB Button to start drift throttle
        drifting = True
    
    if drifting:
        env.step((driftThrotle, servo))
    else:
        env.step((throtle, servo))

if __name__=="__main__":
    global env
    env = gym.make('DriftCarGazeboContinuous-v0')
    env.reset()
    env.render()

    subprocess.Popen(["rosrun", "joy", "joy_node"])
    rospy.Subscriber('/joy', Joy, callback, queue_size=1)

    print("========================================")
    print("Joystick Controller for Gazebo Drift Car")
    print("========================================\n")
    
    print("X for acceleration")
    print("Y to toggle constant speed")
    print("B for hard reset")

    rospy.spin()

    env.close()