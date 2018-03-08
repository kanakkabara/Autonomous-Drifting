#!/usr/bin/env python
import rospy
import gym
import gym_drift_car
from sensor_msgs.msg import Joy

global throtle, servo, drifting
throtle = 250
driftThrotle = 400
THROTLE_STEP = 2
servo = 0
drifting = False

def callback(data, args):
    env = args[0]
    if data.buttons[1] == 1: #B Button for Reset
        env.reset()
        return
        
    global throtle, servo, drifting
    servo = data.axes[0] * 0.46
    throtle = translate(data.axes[2], -1.0, 1.0, 500, 250)

    if data.buttons[3] == 1: #Y Button for toggle drift throttle state
        drifting = not drifting
    
    if drifting:
        env.step((driftThrotle, servo))
    else:
        env.step((throtle, servo))

if __name__=="__main__":
    env = gym.make('DriftCarGazeboContinuous-v0')
    env.reset()
    env.render()

    subprocess.Popen(["rosrun", "joy", "joy_node"])
    rospy.Subscriber('/joy', Joy, callback, (env))

    print("========================================")
    print("Joystick Controller for Gazebo Drift Car")
    print("========================================\n")
    
    print("X for acceleration")
    print("Y to toggle constant speed")
    print("B for hard reset")

    rospy.spin()

    env.close()

def translate(value, leftMin, leftMax, rightMin, rightMax):
    leftSpan = leftMax - leftMin
    rightSpan = rightMax - rightMin
    valueScaled = float(value - leftMin) / float(leftSpan)
    return rightMin + (valueScaled * rightSpan)