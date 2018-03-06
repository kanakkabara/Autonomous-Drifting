#!/usr/bin/env python
import rospy
import os
import signal
import subprocess
import time
from sensor_msgs.msg import Joy
from std_msgs.msg import Int8, Float64, Float64MultiArray

from xbee import XBee
import serial
import struct

global throtle, servo, drifting, pub
throtle = 80.0
driftThrotle = 85.0
THROTLE_STEP = 0.5

servo = 0.0

drifting = False

pub = None

def translate(value, leftMin, leftMax, rightMin, rightMax):
    leftSpan = leftMax - leftMin
    rightSpan = rightMax - rightMin
    valueScaled = float(value - leftMin) / float(leftSpan)
    return rightMin + (valueScaled * rightSpan)

def callback(data, args):
    # print(data)
    global throtle, servo, drifting
    servo = data.axes[0] * 0.46

    throtle = translate(data.axes[2], -1.0, 1.0, 98, 80)

    # if data.buttons[2] == 1: #X Button for Accelerate
    #     throtle = throtle + THROTLE_STEP
    #     if throtle > 179.0:
    #         throtle = 179.0
    # else:
    #     throtle = throtle - THROTLE_STEP
    #     if throtle < 80:
    #         throtle = 80.0

    if data.buttons[1] == 1: #B Button for Reset
        throtle = 40.0
        servo = 0.0
        sendAction(throtle, servo)
        return

    if data.buttons[3] == 1: #Y Button for toggle drift throttle state
        drifting = not drifting
    
    if drifting:
        sendAction(driftThrotle, servo)
    else:
        sendAction(throtle, servo)

    # response = xbee.wait_read_frame()
    # handleXbeeData(response)

def handleXbeeData(response):
    try:
        stateArray = Float64MultiArray()
        for i in range(0, len(response['rf_data']), 4):
            stateArray.data.append(struct.unpack('f',response['rf_data'][i:i+4])[0])
        print(stateArray.data)

        global pub
        pub.publish(stateArray)
    except Exception as e:
        print(e)

def sendAction(throtle, servo):
    print((throtle, servo))
    packed_data = struct.Struct('f f').pack(*(throtle, servo))
    ba = bytearray(packed_data)  
    xbee.send('tx', frame='A', dest_addr=b'\x00\x00', data=ba, options=b'\x04')

if __name__=="__main__":
    PORT = "/dev/ttyUSB1"
    BAUD_RATE = 57600

    ser = serial.Serial(PORT, BAUD_RATE)
    # Async
    xbee = XBee(ser, escaped=True, callback=handleXbeeData)
    
    # Sync
    # xbee = XBee(ser, escaped=True)

    time.sleep(7)

    tmp = os.popen("ps -Af").read()
    roscore_count = tmp.count('roscore')
    if roscore_count == 0:
        subprocess.Popen("roscore")
        time.sleep(1)
        print ("Roscore launched!")

    rospy.init_node('drift_car_teleop_joystick')
    subprocess.Popen(["rosrun", "joy", "joy_node"])
    
    global pub
    pub = rospy.Publisher('drift_car/state', Float64MultiArray, queue_size=1) 
    rospy.Subscriber('/joy', Joy, callback, (pub), queue_size=1)

    print("======================================")
    print("Joystick Controller for Real Drift Car")
    print("======================================\n")
    
    print("X for acceleration")
    print("Y to toggle constant speed")
    print("B for hard reset")

    rospy.spin()
    
    xbee.halt()
    ser.close()