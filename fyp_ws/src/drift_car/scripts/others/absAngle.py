#!/usr/bin/env python
import rospy
from drift_car.msg import IMUData 
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float64
from pyquaternion import Quaternion

def callback(data):
    pub = rospy.Publisher('absAngleData', Float64, queue_size=10)        
    data = data.pose[1].orientation
    q = Quaternion(data.w, data.x, data.y, data.z)
    theta = q.degrees
    print("w", data.w, "x", data.x, "y", data.y, "z", data.z)
    print(theta)
    print('\n')
    

def listener():
    rospy.init_node('absAngle', anonymous=True)
    rospy.Subscriber('gazebo/model_states', ModelStates, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
