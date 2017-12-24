#!/usr/bin/env python
import rospy
from drift_car.msg import IMUData 

def talker():
    pub = rospy.Publisher('IMUData', IMUData, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        imuData = IMUData()
        imuData.header.stamp = rospy.Time.now()
        rospy.loginfo(imuData)
        pub.publish(imuData)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
