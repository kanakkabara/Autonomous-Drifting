#!/usr/bin/env python
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider

import rospy
from std_msgs.msg import Float64MultiArray

def callback(data, args):
    servo = data.data[0]
    left = args[0]
    right = args[1]
    
    if(servo == -1000):
        # args.set_val(0)
        left.set_width(0)
        right.set_width(0)     
        return
    # args.set_val(servo)

    if servo > 0:
        left.set_width(0)
        right.set_width(servo)
    else:
        right.set_width(0)
        left.set_width(servo)
    
    plt.draw()

def steerBar(ax, value):
    bar = ax.barh(0, value, align='center', color='red')
    ax.get_yaxis().set_visible(False)
    bar.patches[0].set_width(0)
    return bar.patches[0]

if __name__ == '__main__':
    rospy.init_node("drift_car_steering_visualizer")

    fig, axes = plt.subplots(ncols=2, sharey=True)
    fig.set_figheight(0.9)
    plt.subplots_adjust(left=0.05, right=0.95, top=0.9, bottom=0.3, wspace=0)
    left = steerBar(axes[0], -0.4363)
    right = steerBar(axes[1], 0.4363)
    rospy.Subscriber('drift_car/action', Float64MultiArray, callback, (left, right))

    # fig, ax = plt.subplots()
    # fig.set_figheight(0.5)
    # plt.subplots_adjust(left=0.2, right=0.9, top=0.9, bottom=0.15)
    # slider = Slider(ax, 'Steering Angle', -0.4363, 0.4363, valinit=0, dragging=False)
    # rospy.Subscriber('drift_car/action', Float64MultiArray, callback, (slider))

    plt.show()