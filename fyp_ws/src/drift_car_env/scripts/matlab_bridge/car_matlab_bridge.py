#!/usr/bin/env python
import time
import math
import struct
import signal

import rospy
from sensor_msgs.msg import Joy, Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray, Header
from geometry_msgs.msg import Vector3
from drift_car.srv import GetLatestState, GetLatestStateResponse

from imu_merger import ImuMerger

from xbee import XBee
import serial

def translate(value, leftMin, leftMax, rightMin, rightMax):
    leftSpan = leftMax - leftMin
    rightSpan = rightMax - rightMin
    valueScaled = float(value - leftMin) / float(leftSpan)
    return rightMin + (valueScaled * rightSpan)

def actionCallback(data, args):
    time.sleep(0.05)
    action = data.data[0]
    takenOn = data.data[1]

    if(action == -1000):
        rospy.loginfo('Resetting Env . . . \n\n')
        sendAction(stopThrottle, 0)
        return
    sendAction(throttle, action)

def sigStopHandler(signum, frame):
    global tempStop
    print("Ctrl+Z detected, stopping car . . . ")
    sendAction(stopThrottle, 0)
    tempStop = not tempStop

def sendAction(throttle, servo):
    # TODO decide what to do if temporarily stopped....
    # if tempStop:        
    #     return

    # rospy.loginfo(rospy.get_caller_id() + 'Action: %s , %s', throttle, servo)            
    packed_data = struct.Struct('f f').pack(*(throttle, servo))
    ba = bytearray(packed_data)  
    xbee.send('tx', frame='A', dest_addr=b'\x00\x00', data=ba, options=b'\x04')

def handleXbeeData(response):
    rate = rospy.Rate(125)    
    # if tempStop:        
        # rate.sleep() 
        # return
    try:
        data = []
        # Expected State = xDot (m/s), yDot (m/s), thetaDot (degrees/s), throttle, servo
        for i in range(0, len(response['rf_data']), 4):
            data.append(struct.unpack('f',response['rf_data'][i:i+4])[0])
        EPOCH_BASE = 1520000000  
        TIME_OFFSET = 30659 + 3924   

        imu = Imu()
        imu.linear_acceleration = Vector3(data[0], data[1], data[2])
        imu.angular_velocity = Vector3(data[3], data[4], data[5])
        imu.header = Header()
        imu.header.stamp = rospy.Time(EPOCH_BASE + int(data[6] + TIME_OFFSET), int(data[7] * 1000))
        imuPub.publish(imu)

        odomData = rospy.wait_for_message('/odometry/filtered', Odometry, timeout=1)
        twist = odomData.twist.twist      
        velx = twist.linear.x
        vely = twist.linear.y
        carTangentialSpeed = math.sqrt(velx ** 2 + vely ** 2)

        stateArray = Float64MultiArray()
        stateArray.data.append(velx)
        stateArray.data.append(vely)
        stateArray.data.append(carTangentialSpeed)        
        stateArray.data.append(twist.angular.z)

        # latestState = stateArray.data
        # # Add actions before publishing state
        # # stateArray.data.extend(data[-2:])

        print(stateArray.data)
        # pub.publish(stateArray)
        # rate.sleep()        
    except Exception as e:
        print(e)

def handleGetLatestState(req):
    print(latestState)    
    return GetLatestStateResponse(latestState)

def joystickCallback(data, args):
    servo = data.axes[3] * 0.46
    throtleSend = translate(data.axes[2], -1.0, 1.0, throttle, 90)

    if data.buttons[1] == 1: # B Button for Reset
        sendAction(stopThrottle, 0)
        return

    global drifting
    if data.buttons[4] == 1: # LB Button to stop drift throttle
        drifting = False
    if data.buttons[5] == 1: # RB Button to start drift throttle
        drifting = True
    
    if drifting:
        sendAction(throttle, servo)
    else:
        sendAction(throtleSend, servo)

    time.sleep(0.05)

if __name__=="__main__":  
    throttle = 102
    stopThrottle = 30.0
    latestState = None
    tempStop = False
    drifting = False

    # rospy.set_param('use_sim_time', 'false')
    rospy.init_node('drift_car_matlab_bridge')
    pub = rospy.Publisher('drift_car/state', Float64MultiArray, queue_size=1) 
    imuPub = rospy.Publisher('drift_car/imu_data_actual', Imu, queue_size=100000) 
    # rospy.Subscriber('drift_car/action', Float64MultiArray, actionCallback, (pub))
    rospy.Subscriber('/joy', Joy, joystickCallback, (pub), queue_size=1)
    rospy.Service('drift_car/get_latest_state', GetLatestState, handleGetLatestState)

    imu_merger = ImuMerger('imu', '/drift_car/imu_data_actual')

    PORT = "/dev/ttyUSB0"
    BAUD_RATE = 57600
    ser = serial.Serial(PORT, BAUD_RATE)
    # Async
    xbee = XBee(ser, escaped=True, callback=handleXbeeData)
    # Sync
    # xbee = XBee(ser, escaped=True)

    print("Handshake . . " + str(time.time() % 10000000))
    packed_data = struct.Struct('f f').pack(*(time.time() % 10000000, 0))
    ba = bytearray(packed_data)  
    xbee.send('tx', frame='A', dest_addr=b'\x00\x00', data=ba, options=b'\x04')

    # To stop car on SIGSTOP signal
    signal.signal(signal.SIGTSTP, sigStopHandler)
    # Keep node running
    rospy.spin()
    # Reset car after program is killed
    sendAction(stopThrottle, 0)
    # Close Xbee/Serial connections
    xbee.halt()
    ser.close()