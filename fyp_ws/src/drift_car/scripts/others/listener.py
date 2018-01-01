#!/usr/bin/env python
import rospy
from drift_car.msg import IMUData 
from gazebo_msgs.msg import ModelStates

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + ' position: %s', data.pose[1].position)

def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    # rospy.Subscriber('IMUData', IMUData, callback)
    rospy.Subscriber('gazebo/model_states', ModelStates, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
