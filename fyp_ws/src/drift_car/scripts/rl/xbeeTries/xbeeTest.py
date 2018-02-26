from xbee.thread import XBee
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
           print (response)
           while True:
               xbee.tx(dest_addr=b'\x00\x00', data='Received by remote')
               time.sleep(1)
        except KeyboardInterrupt:
           break
           ser.close()
