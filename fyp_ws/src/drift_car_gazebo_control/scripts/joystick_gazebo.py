#!/usr/bin/env python
import rospy
import os
import signal
import subprocess
import time
from std_msgs.msg import Float64
from sensor_msgs.msg import Joy
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelStates, ModelState

throtle1 = rospy.Publisher('/drift_car/joint1_position_controller/command', Float64, queue_size = 1)
throtle2 = rospy.Publisher('/drift_car/joint2_position_controller/command', Float64, queue_size = 1)
steer1 = rospy.Publisher('/drift_car/joint3_position_controller/command', Float64, queue_size = 1)
steer2 = rospy.Publisher('/drift_car/joint4_position_controller/command', Float64, queue_size = 1)

global throtle, servo, drifting
throtle = 250
driftThrotle = 400
THROTLE_STEP = 2

servo = 0

drifting = False

def callback(data):
    global throtle, servo, drifting
    # if data.axes[1] >= 0:
    servo = data.axes[0] * 0.46
    # else:
        # servo = 0
    
    if data.buttons[2] == 1: #X Button for Accelerate
        throtle = throtle + THROTLE_STEP
        if throtle > 500:
            throtle = 500
    else:
        throtle = throtle - THROTLE_STEP
        if throtle < 250:
            throtle = 250

    if data.buttons[1] == 1: #B Button for Reset
        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            reset_pose = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            nullPosition = ModelState()
            nullPosition.model_name = "drift_car"
            nullPosition.pose.position.x = 0
            nullPosition.pose.position.y = 0
            nullPosition.pose.position.z = 0.1
            reset_pose(nullPosition)
        except (rospy.ServiceException) as e:
            print ("/gazebo/set_model_state service call failed")

    if data.buttons[3] == 1: #Y Button for toggle drift throttle state
        drifting = not drifting
    
    if drifting:
        print((driftThrotle, servo))
    else:
        print((throtle, servo))

    cmd = Float64()
    if drifting:
        cmd.data = driftThrotle
    else:
        cmd.data = throtle  
    throtle1.publish(cmd)
    throtle2.publish(cmd)

    cmd = Float64()
    cmd.data = servo  
    steer1.publish(cmd)
    steer2.publish(cmd)

if __name__=="__main__":
    tmp = os.popen("ps -Af").read()
    roscore_count = tmp.count('roscore')
    if roscore_count == 0:
        subprocess.Popen("roscore")
        time.sleep(1)
        print ("Roscore launched!")
    
    rospy.init_node('drift_car_teleop_joystick')
    rospy.Subscriber('/joy', Joy, callback)
    
    subprocess.Popen(["roslaunch", "drift_car_gazebo", "drift_car.launch"])
    time.sleep(10)
    subprocess.Popen(["roslaunch", "drift_car_gazebo_control", "drift_car_control.launch"])
    time.sleep(5)
    print ("Gazebo launched!")      
    subprocess.Popen(["rosrun", "joy", "joy_node"])
    subprocess.Popen(["gzclient"])

    print("========================================")
    print("Joystick Controller for Gazebo Drift Car")
    print("========================================\n")
    
    print("X for acceleration")
    print("Y to toggle constant speed")
    print("B for hard reset")

    rospy.spin()

    tmp = os.popen("ps -Af").read()
    gzclient_count = tmp.count('gzclient')
    gzserver_count = tmp.count('gzserver')
    roscore_count = tmp.count('roscore')
    python_count = tmp.count('python')
    rosmaster_count = tmp.count('rosmaster')


    if gzclient_count > 0:
        os.system("killall -9 gzclient")
    if gzserver_count > 0:
        os.system("killall -9 gzserver")
    if roscore_count > 0:
        os.system("killall -9 roscore")
    if rosmaster_count > 0:
        os.system("killall -9 rosmaster")
    if python_count > 0:
        os.system("killall -9 python")
    
    if (gzclient_count or gzserver_count or roscore_count or python_count >0):
        os.wait()