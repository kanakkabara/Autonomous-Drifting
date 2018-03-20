#!/usr/bin/env python
import subprocess
import struct
import math

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Int8, Float64, Float64MultiArray

from xbee import XBee
import serial

global drifting, pub
driftThrotle = 100.0
drifting = False
pub = None

def translate(value, leftMin, leftMax, rightMin, rightMax):
    leftSpan = leftMax - leftMin
    rightSpan = rightMax - rightMin
    valueScaled = float(value - leftMin) / float(leftSpan)
    return rightMin + (valueScaled * rightSpan)

def callback(data, args):
    # print(data)
    servo = data.axes[3] * 0.46
    #Convert to degrees 
    servo = translate(servo, -0.436, 0.436, 65, 115)
    throtle = translate(data.axes[2], -1.0, 1.0, 98, 80)

    if data.buttons[1] == 1: #B Button for Reset
        sendAction(30.0, 0)
        return

    global drifting
    if data.buttons[4] == 1: # LB Button to stop drift throttle
        drifting = False
    if data.buttons[5] == 1: # RB Button to start drift throttle
        drifting = True
    
    if drifting:
        sendAction(driftThrotle, servo)
    else:
        sendAction(throtle, servo)

    # response = xbee.wait_read_frame()
    # handleXbeeData(response)

def handleXbeeData(response):
    try:
        stateArray = Float64MultiArray()
        # Expected State = xDot (m/s), yDot (m/s), thetaDot (degrees/s), throttle, servo
        for i in range(0, len(response['rf_data']), 4):
            stateArray.data.append(struct.unpack('f',response['rf_data'][i:i+4])[0])
        # Convert to rads/s
        stateArray.data[2] = stateArray.data[2]/180
        
        # Add tangential speed to state
        velx = stateArray.data[0]
        vely = stateArray.data[1]
        carTangentialSpeed = math.sqrt(velx ** 2 + vely ** 2)
        stateArray.data.insert(3, carTangentialSpeed)
        
        global pub
        if pub is not None:
            # Expected State = xDot (m/s), yDot (m/s), thetaDot (rads/s), tangentialSpeed, throttle, servo
            print(stateArray.data)
            pub.publish(stateArray)
    except Exception as e:
        print(e)

def sendAction(throtle, servo):
    print((throtle, servo))
    packed_data = struct.Struct('f f').pack(*(throtle, servo))
    ba = bytearray(packed_data)  
    xbee.send('tx', frame='A', dest_addr=b'\x00\x00', data=ba, options=b'\x04')

if __name__=="__main__":
    PORT = "/dev/ttyUSB0"
    BAUD_RATE = 57600

    ser = serial.Serial(PORT, BAUD_RATE)
    # Async
    xbee = XBee(ser, escaped=True, callback=handleXbeeData)
    
    # Sync
    # xbee = XBee(ser, escaped=True)

    rospy.init_node('drift_car_teleop_joystick')
    subprocess.Popen(["rosrun", "joy", "joy_node"])
    
    global pub
    pub = rospy.Publisher('drift_car/state', Float64MultiArray, queue_size=1) 
    rospy.Subscriber('/joy', Joy, callback, (pub), queue_size=1)

    print("======================================")
    print("Joystick Controller for Real Drift Car")
    print("======================================\n")
    
    print("RT for acceleration")
    print("LB/RB to toggle constant speed")
    print("B for hard reset")

    rospy.spin()
    
    xbee.halt()
    ser.close()