import rospy
import time
import subprocess
import message_filters
from sensor_msgs.msg import Imu
from std_msgs.msg import Header

class ImuMerger:
    def __init__(self, *queues):
        self.estimateProcess = subprocess.Popen(["roslaunch", "drift_car_gazebo", "drift_car_ekf.launch"])
        
        self.queues = queues
        self.sources = []
        self.pubs = []
        for queue in queues:
            self.sources.append(message_filters.Subscriber(queue, Imu))
            self.pubs.append(rospy.Publisher(queue + '/sync', Imu, queue_size = 10000))

        self.ts = message_filters.ApproximateTimeSynchronizer(self.sources, 10, 1)
        self.ts.registerCallback(self.callback)

    def callback(self, *args):
        i = 0
        now = rospy.Time.from_sec(time.time())
        for syncData in args:
            syncData.header = Header()
            syncData.header.stamp = now
            syncData.header.frame_id = "base_link"
            self.pubs[i].publish(syncData)
            i = i + 1        