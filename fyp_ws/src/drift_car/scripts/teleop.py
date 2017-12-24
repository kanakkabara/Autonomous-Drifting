#!/usr/bin/env python
import rospy

from std_msgs.msg import Float64

import sys, select, termios, tty

msg = """
Reading from the keyboard  and Publishing to Float64!
---------------------------
Moving around:
        w
a       s       d

CTRL-C to quit
"""

THROTTLE_STEP = 50
STEER_STEP = 0.1
speed = 300
steeringAngle = 0

def getKey():
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key

if __name__=="__main__":
    	settings = termios.tcgetattr(sys.stdin)
	
	throtle1 = rospy.Publisher('/drift_car/joint1_position_controller/command', Float64, queue_size = 1)
	throtle2 = rospy.Publisher('/drift_car/joint2_position_controller/command', Float64, queue_size = 1)
	
	steer1 = rospy.Publisher('/drift_car/joint3_position_controller/command', Float64, queue_size = 1)
	steer2 = rospy.Publisher('/drift_car/joint4_position_controller/command', Float64, queue_size = 1)
	
	rospy.init_node('drift_car_teleop_keyboard')

	try:
	        print (msg)
	        cmd = Float64()
	        cmd.data = steeringAngle  
		steer1.publish(cmd)
                steer2.publish(cmd)
	        
	        while(1):
			key = getKey()
			cmd = Float64()
			if key == 'w' or key == 's':
                                if key == 'w':
                                        speed = speed + THROTTLE_STEP
                                if key == 's':  
                                        speed = speed - THROTTLE_STEP
        			cmd.data = speed
	        		throtle1.publish(cmd)
	        		throtle2.publish(cmd)
			elif key == 'd' or key == 'a':                
                                if key == 'd':
                                        steeringAngle = steeringAngle - STEER_STEP             
                                        if steeringAngle < -0.785:
                                                steeringAngle = -0.785
                                if key == 'a':  
                                        steeringAngle = steeringAngle + STEER_STEP
                                        if steeringAngle > 0.785:
                                                steeringAngle = 0.785
        			cmd.data = steeringAngle  
	        		steer1.publish(cmd)
	        		steer2.publish(cmd)
			elif (key == '\x03'):
				break
			print(speed, steeringAngle)
	except:
		print e

	finally:
        	cmd = Float64()
        	cmd.data = 0
        	throtle1.publish(cmd)
		throtle2.publish(cmd)
		steer1.publish(cmd)
		steer2.publish(cmd)
		
  		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
