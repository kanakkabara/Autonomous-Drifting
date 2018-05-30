#!/usr/bin/env python
import rospy
import gym
import gym_drift_car
import sys, select, termios, tty

msg = """
Reading from the keyboard  and Publishing to Float64!
---------------------------
Moving around:
        w
a       s       d

CTRL-C to quit
"""

THROTTLE_STEP = 10
STEER_STEP = 0.1
throttle = 1650
servo = 0

def getKey():
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key

if __name__=="__main__":
    	settings = termios.tcgetattr(sys.stdin)
	env = gym.make('CollisionContinuousGazebo-v0') 
	env.reset()
	env.render()

	try:
	        while(1):
			key = getKey()
			if key == 'w' or key == 's':
                                if key == 'w':
                                        throttle = throttle + THROTTLE_STEP
                                if key == 's':  
                                        throttle = throttle - THROTTLE_STEP
			elif key == 'd' or key == 'a':                
                                if key == 'd':
                                        servo = servo - STEER_STEP             
                                        if servo < -0.436:
                                                servo = -0.436
                                if key == 'a':  
                                        servo = servo + STEER_STEP
                                        if servo > 0.436:
                                                servo = 0.436
			elif (key == 'r'):
				env.reset()
				servo = 0
				continue
			elif (key == '\x03'):
				break
			print(throttle, servo)
			state, reward, done, extra = env.step((throttle, servo))
			if done:
				env.reset()
	except Exception as e:
		print e

	finally:
		env.reset()
		env.close()
		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
