#!/usr/bin/env python
import time
import math
import struct
import signal

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Int8, Float64, Float64MultiArray, MultiArrayLayout
from drift_car.srv import GetLatestState, GetLatestStateResponse

from xbee import XBee
import serial

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
        stateArray = Float64MultiArray()
        # Expected State = xDot (m/s), yDot (m/s), thetaDot (degrees/s), throttle, servo
        for i in range(0, len(response['rf_data']), 4):
            stateArray.data.append(struct.unpack('f',response['rf_data'][i:i+4])[0])
        #Convert to rads/s
        stateArray.data[2] = stateArray.data[2]/180

        # Add tangential speed to state
        velx = stateArray.data[0]
        vely = stateArray.data[1]
        carTangentialSpeed = math.sqrt(velx ** 2 + vely ** 2)
        stateArray.data.insert(3, carTangentialSpeed)

        # Drop actions before publishing state
        # stateArray.data = stateArray.data[:-2]

        print(stateArray.data)
        latestState = stateArray.data
        
        pub.publish(stateArray)
        rate.sleep()        
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
    global drifting, tempStop, stopThrottle
    throttle = 102
    stopThrottle = 30.0
    latestState = None
    tempStop = False
    drifting = False

    rospy.init_node('drift_car_matlab_bridge')
    pub = rospy.Publisher('drift_car/state', Float64MultiArray, queue_size=1) 
    rospy.Subscriber('drift_car/action', Float64MultiArray, callback, (pub))
    rospy.Service('drift_car/get_latest_state', GetLatestState, handleGetLatestState)
    rospy.Subscriber('/joy', Joy, joystickCallback, (pub), queue_size=1)
  
    PORT = "/dev/ttyUSB0"
    BAUD_RATE = 57600
    ser = serial.Serial(PORT, BAUD_RATE)
    # Async
    xbee = XBee(ser, escaped=True, callback=handleXbeeData)
    # Sync
    # xbee = XBee(ser, escaped=True)

    # To stop car on SIGSTOP signal
    signal.signal(signal.SIGTSTP, sigStopHandler)
    # Keep node running
    rospy.spin()
    # Reset car after program is killed
    sendAction(stopThrottle, 0)
    # Close Xbee/Serial connections
    xbee.halt()
    ser.close()