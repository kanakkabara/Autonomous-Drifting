#!/usr/bin/env python
import rospy
import gym
import gym_drift_car

import matplotlib.pyplot as plt
from matplotlib import style

import sys, select, termios, tty

msg = """
Reading from the keyboard  and Publishing to Float64!
---------------------------
Moving around:
        w
a       s       d

CTRL-C to quit
"""

env = gym.make('DriftCarGazeboEnv-v0')

def getKey():
	tty.setraw(sys.stdin.fileno())
	select.select([sys.stdin], [], [], 0)
	key = sys.stdin.read(1)
	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key

if __name__=="__main__":
    	settings = termios.tcgetattr(sys.stdin)
	try:
        	fig = plt.figure(figsize=(10, 10))
        	ax1 = fig.add_subplot(211)
                ax1.set_xlabel('Number of steps')
                ax1.set_ylabel('Total Reward')
                plt.ion()
                fig.show()
                fig.canvas.draw()
                
	        runningReward = [0]
                action = 3
                env.reset()
	        while(1):
        	        env.render()
			key = getKey()
			if key == 'r':
                                runningReward = [0]
                                action = 3
                                env.reset()
                                env.step(action)
			elif key == 'd' or key == 'a':                
                                if key == 'd':
                                        action = action - 1             
                                        if action < 0:
                                                action = 0
                                if key == 'a':  
                                        action = action + 1
                                        if action > 6:
                                                action = 6
                                next_state, reward, done, _ = env.step(action)
                                runningReward.append(runningReward[-1] + reward)
			elif (key == '\x03'):
				break
			else:
		                next_state, reward, done, _ = env.step(action)
                                runningReward.append(runningReward[-1] + reward)
                                
                        ax1.clear()
                        ax1.plot(runningReward)
                        fig.canvas.draw()

	except Exception as e: 
		print e

	finally:
  		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)	
