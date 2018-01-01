#!/usr/bin/env python
import rospy
from drift_car.msg import IMUData 
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float64

def callback(data):
    pub = rospy.Publisher('absAngle', Float64, queue_size=10)        
    pub.publish(abs(data.pose[1].orientation.w))

def listener():
    rospy.init_node('absAngle', anonymous=True)
    rospy.Subscriber('gazebo/model_states', ModelStates, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
