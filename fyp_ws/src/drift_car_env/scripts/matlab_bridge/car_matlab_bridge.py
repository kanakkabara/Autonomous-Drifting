#!/usr/bin/env python
import rospy
import os
import signal
import subprocess
import time
import math
from sensor_msgs.msg import Joy
from std_msgs.msg import Int8, Float64, Float64MultiArray, MultiArrayLayout
from drift_car.srv import GetLatestState, GetLatestStateResponse

from xbee import XBee
import serial
import struct

throtle = 100
latestState = None
global count 
count = 0    

def translate(value, leftMin, leftMax, rightMin, rightMax):
    leftSpan = leftMax - leftMin
    rightSpan = rightMax - rightMin
    valueScaled = float(value - leftMin) / float(leftSpan)
    return rightMin + (valueScaled * rightSpan)

def callback(data, args):
    time.sleep(0.05)
    action = data.data[0]
    takenOn = data.data[1]

    if takenOn == 0: # Action to be taken on the Car
        if(action == -1000):
            rospy.loginfo('Resetting Env . . . \n\n')
            sendAction(40, 90)
            return
        action = translate(action, -0.436, 0.436, 115, 65)
        sendAction(throtle, action)

def sendAction(throtle, servo):
    global count 
    rospy.loginfo(rospy.get_caller_id() + ' Count: %s, Action: %s , %s', count, throtle, servo)            
    packed_data = struct.Struct('f f').pack(*(throtle, servo))
    ba = bytearray(packed_data)  
    xbee.send('tx', frame='A', dest_addr=b'\x00\x00', data=ba, options=b'\x04')
    count = count + 1

def handleXbeeData(response):
    rate = rospy.Rate(125)    
    try:
        stateArray = Float64MultiArray()
        for i in range(0, len(response['rf_data']), 4):
            stateArray.data.append(struct.unpack('f',response['rf_data'][i:i+4])[0])
        stateArray.data[2] = stateArray.data[2]/180
        velx = stateArray.data[0]
        vely = stateArray.data[1]
        carTangentialSpeed = math.sqrt(velx ** 2 + vely ** 2)
        stateArray.data.append(carTangentialSpeed)

        print(stateArray.data)
        print("Counter: "+ str(count) +" \n")
        # stateArray.data = stateArray.data[:-2]
        latestState = stateArray.data
        pub.publish(stateArray)
        rate.sleep()        
    except Exception as e:
        print(e)

def handleGetLatestState(req):
    print(latestState)    
    return GetLatestStateResponse(latestState)

if __name__=="__main__":  
    tmp = os.popen("ps -Af").read()
    roscore_count = tmp.count('roscore')
    if roscore_count == 0:
        subprocess.Popen("roscore")
        time.sleep(1)
        print ("Roscore launched!")

    rospy.init_node('drift_car_matlab_bridge')
    pub = rospy.Publisher('matlab_bridge/state', Float64MultiArray, queue_size=1) 
    rospy.Subscriber('matlab_bridge/action', Float64MultiArray, callback, (pub))
    rospy.Service('matlab_bridge/get_latest_state', GetLatestState, handleGetLatestState)
  
    PORT = "/dev/ttyUSB0"
    BAUD_RATE = 57600

    ser = serial.Serial(PORT, BAUD_RATE)
    # Async
    xbee = XBee(ser, escaped=True, callback=handleXbeeData)
    
    # Sync
    # xbee = XBee(ser, escaped=True)

    # while id < 1000:
    #     print(id)
    #     stateArray = Float64MultiArray()
    #     stateArray.layout = MultiArrayLayout()
    #     stateArray.layout.data_offset = id
    #     stateArray.data.append(id)
    #     id = id + 1
    #     pub.publish(stateArray)

    rospy.spin()

    sendAction(0, 90)
    
    xbee.halt()
    ser.close()