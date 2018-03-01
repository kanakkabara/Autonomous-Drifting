#!/usr/bin/env python
from xbee.thread import XBee
import messages_pb2
import serial
import time

PORT = '/dev/ttyUSB0'
BAUD_RATE = 9600

# Open serial port
ser = serial.Serial(PORT, BAUD_RATE)

# Create API object
xbee = XBee(ser)

# Continuously read and print packets
while True:
        try:
            response = xbee.wait_read_frame()
            action = messages_pb2.Action()
            action.ParseFromString(response['rf_data'].decode("utf-8"))
            print(action.throttle)
            print(action.steering)

            # state = messages_pb2.State()
            # print(state.ParseFromString(response['rf_data']))

        #   while True:
            # action = messages_pb2.Action()
            # action.throttle = 40
            # action.steering = 70
            # xbee.tx(dest_addr=b'\x00\x00', data=action.SerializeToString())

            # time.sleep(1)
        except KeyboardInterrupt:
            break
            ser.close()
