#!/usr/bin/env python
import gym
from gym import error, spaces, utils
from gym.utils import seeding

import rospy
from std_msgs.msg import Float64
from std_srvs.srv import Empty
from sensor_msgs.msg import Imu
from gazebo_msgs.msg import ModelStates, ModelState
from gazebo_msgs.srv import SetModelState

import numpy as np

import os
import math
import signal
import subprocess
import time
from os import path

class GazeboEnv(gym.Env):
        metadata = {'render.modes': ['human']}
        # Default state: x, y, i, j, k, w, xdot, ydot, thetadot, s. 
        DEFAULT_STATE_INFO = {'state_size': 10, 'include_theta': True, 'include_position': True,
                'include_tangential_speed': True}

        def __init__(self, continuous=False, state_info=DEFAULT_STATE_INFO):
                rospy.init_node('gazebo_drift_car_gym', anonymous=True)
                
                self.gazeboProcess = subprocess.Popen(["roslaunch", "drift_car_gazebo", "drift_car.launch"])
                time.sleep(10)
                self.controlProcess = subprocess.Popen(["roslaunch", "drift_car_gazebo_control", "drift_car_control.launch"])
                time.sleep(5)
                                
                print ("Gazebo launched!")      
                
                self.gzclient_pid = 0
                self.throtle1 = rospy.Publisher('/drift_car/joint1_position_controller/command', Float64, queue_size = 1)
                self.throtle2 = rospy.Publisher('/drift_car/joint2_position_controller/command', Float64, queue_size = 1)
                self.steer1 = rospy.Publisher('/drift_car/joint3_position_controller/command', Float64, queue_size = 1)
                self.steer2 = rospy.Publisher('/drift_car/joint4_position_controller/command', Float64, queue_size = 1)
        
                self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
                self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
                self.reset_proxy = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
                
                #Reward related
                self.reward_range = (-np.inf, np.inf)

                #Action related
                self.continuous = continuous
                if continuous:
                        high = np.array([0.436])
                        self.action_space = spaces.Box(-high, high)
                else:
                        self.degreeMappings = [65, 75, 85, 90, 95, 105, 115]
                        self.radianMappings = [-0.436, -0.261799, -0.0872665, 0, 0.0872665, 0.261799, 0.436] 
                        self.action_space = spaces.Discrete(len(self.degreeMappings))
                
                #State related
                self.state_info= state_info
                high = np.ones(self.state_info['state_size']) * np.finfo(np.float32).max  
                self.observation_space = spaces.Box(-high, high)   
                
                self._seed()
                
                self.previous_action = -1
                self.previous_imu = {}
                self.previous_pos = self.getPosData()
                  
                # Learning Parameters
                self.radius = 3
                self.throttle = 415      
                self.maxDeviationFromCenter = 4
                
        def _seed(self, seed=None):
                self.np_random, seed = seeding.np_random(seed)
                return [seed] 
                
        def _step(self, action):
                #TODO can look into mirroring joints to make sure the wheels spin and turn tgt                
                
                self.unpausePhysics()

                if isinstance(action, tuple):
                        self.throtle1.publish(action[0])
		        self.throtle2.publish(action[0])
                        action = action[1]
                else:
                        self.throtle1.publish(self.throttle)
		        self.throtle2.publish(self.throttle)
                
                if self.continuous:
                        self.steer1.publish(action)
                        self.steer2.publish(action)
                else:
                        self.steer1.publish(self.radianMappings[action])
                        self.steer2.publish(self.radianMappings[action])                

                posData = self.getPosData()
                #imuData = self.getIMUData()                

                self.pausePhysics()

                state = self.getState(posData)
                reward = self.getRewardExponential(posData)
                done = self.isDone(posData)
              
                #self.previous_imu = imuData
                self.previous_pos = posData     
                self.previous_action = action
                return state, reward, done, {}
                
        def getState(self, posData):
                velx = posData.twist[1].linear.x
                vely = posData.twist[1].linear.y
                carTangentialSpeed = math.sqrt(velx ** 2 + vely ** 2)
                carAngularVel = posData.twist[1].angular.z

                state = [velx, vely, carAngularVel]

                if 'include_theta' in self.state_info and self.state_info['include_theta']:
                    # i, j, k, w appended to state.
                    theta_state = [posData.pose[1].orientation.x, posData.pose[1].orientation.y, 
                            posData.pose[1].orientation.z, posData.pose[1].orientation.w]
                    state = theta_state + state

                if 'include_position' in self.state_info and self.state_info['include_position']:
                    # x, y appended to state.
                    pos_state = [posData.pose[1].position.x, posData.pose[1].position.y]
                    state = pos_state + state

                if 'include_tangential_speed' in self.state_info and self.state_info['include_tangential_speed']:
                    state += [carTangentialSpeed]

                return np.array(state)

        def getRewardExponential(self, posData):
                desiredTangentialSpeed = 5          # Tangential speed with respect to car body.
                # desiredNormalSpeed  = 0           # Perfect circular motion
                desiredAngularVel = 4 

                velx = posData.twist[1].linear.x
                vely = posData.twist[1].linear.y
                carTangentialSpeed = math.sqrt(velx ** 2 + vely ** 2)
                carAngularVel = posData.twist[1].angular.z

                sigma = 0.5
                #deviationMagnitude = (carSideVel - desiredSideVel)**2 + \
                #                (carForwardVel - desiredForwardVel)**2 + \
                #                (carAngularVel - desiredAngularVel)**2
                
                deviationMagnitude = (desiredTangentialSpeed - carTangentialSpeed)**2 + \
                                (carAngularVel - desiredAngularVel)**2


                return math.exp(-deviationMagnitude/(2 * sigma**2)) - 1
        
        def getRewardPotentialBased(self, action, posData):
                reward = 0.0
                
                largeActionDeltaPenalty = -1
                actionDelta = self.previous_action - action
                actionDeltaPenalty = (actionDelta ** 2) * largeActionDeltaPenalty
                
                # Window for potential based reward.
                angleRewardWindow = 0.1
                
                # Calculate the potential reward based on polar angle difference.
                prevAngle = self.previous_pos.pose[1].orientation.w
                currAngle = posData.pose[1].orientation.w
                if currAngle > prevAngle or abs(prevAngle - currAngle) <= angleRewardWindow:
                        anglePotentialReward = 10
                else: 
                        anglePotentialReward = -10
                        
                # Calculate the potential reward based on circular path.
                x = posData.pose[1].position.x
                y = posData.pose[1].position.y
                deviationPenalty = -(abs((self.radius ** 2) - (x ** 2 + y ** 2)))
                
                reward = actionDeltaPenalty + anglePotentialReward + deviationPenalty
                return reward
             
        def isDone(self, posData):       
                #Done is true if the car ventures too far from the center of the circular drift
                x = posData.pose[1].position.x
                y = posData.pose[1].position.y
                return (self.maxDeviationFromCenter <= ((x ** 2 + y ** 2) ** 0.5))
                
        def _reset(self):
                #print("Reset called")  
                rospy.wait_for_service('/gazebo/set_model_state')
                try:
                    reset_pose = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
                    nullPosition = ModelState()
                    nullPosition.model_name = "drift_car"
                    nullPosition.pose.position.x = 0
                    nullPosition.pose.position.y = 0
                    nullPosition.pose.position.z = 0.1
                    reset_pose(nullPosition)
                except (rospy.ServiceException) as e:
                    print ("/gazebo/set_model_state service call failed")
                #print("Reset done")
                
                self.unpausePhysics()
                posData = self.getPosData()
                # imuData = self.getIMUData()
                self.pausePhysics()

                self.previous_action = -1
                self.previous_imu = {}
                self.previous_pos = posData

                return self.getState(posData)
        
        def _render(self, mode='human', close=False):
                if close:
                    tmp = os.popen("ps -Af").read()
                    proccount = tmp.count('gzclient')
                    if proccount > 0:
                        if self.gzclient_pid != 0:
                            os.kill(self.gzclient_pid, signal.SIGTERM)
                            os.wait()
                    return

                tmp = os.popen("ps -Af").read()
                proccount = tmp.count('gzclient')
                if proccount < 1:
                    subprocess.Popen("gzclient")
                    self.gzclient_pid = int(subprocess.check_output(["pidof","-s","gzclient"]))
                else:
                    self.gzclient_pid = 0
    
        def handleGazeboFailure(self):
                print("Failed too many times, trying to restart Gazebo")
                tmp = os.popen("ps -Af").read()
                gzserver_count = tmp.count('gzserver')
                gzclient_count = tmp.count('gzclient')
                control_count = tmp.count('/usr/bin/python /opt/ros/kinetic/bin/roslaunch drift_car_gazebo_control drift_car_control.launch')               
                
                if gzclient_count > 0:
                    os.system("killall -9 gzclient")
                if gzserver_count > 0:
                    os.system("killall -9 gzserver")    
                if control_count > 0:
                    os.system('pkill -TERM -P {pid}'.format(pid=self.controlProcess.pid))
                
                if (gzclient_count or gzserver_count or control_count > 0):
                    os.wait()
                        
                self.gazeboProcess = subprocess.Popen(["roslaunch", "drift_car_gazebo", "drift_car.launch"])
                time.sleep(10)
                self.controlProcess = subprocess.Popen(["roslaunch", "drift_car_gazebo_control", "drift_car_control.launch"])
                time.sleep(5)
    
        def getIMUData(self):
                #print("Fetching IMU Data")
                failureCount = 0
                imuData = None
                while imuData is None:
                        try:
                                imuData = rospy.wait_for_message('/drift_car/imu_data', Imu, timeout=1)
                        except Exception as e: 
                                failureCount += 1 
                                if failureCount % 10 == 0:
                                        self.handleGazeboFailure()     
                                print(e)
                                pass
                #print("Fetched IMU Data")
                return imuData
                
        def getPosData(self):
                #print("Fetching Pos Data")
                failureCount = 0
                posData = None        
                while posData is None:
                        try:
                                posData = rospy.wait_for_message('/gazebo/model_states', ModelStates, timeout=1)
                                posData.pose[1].orientation.w = abs(posData.pose[1].orientation.w)
                        except Exception as e:
                                failureCount += 1
                                if failureCount % 10 == 0:
                                        self.handleGazeboFailure()          
                                print(e)
                                pass
                #print("Fetched Pos Data")
                return posData
    
        def pausePhysics(self): 
                #print("Pause called")                        
                rospy.wait_for_service('/gazebo/pause_physics')
                try:
                    self.pause()
                except (rospy.ServiceException) as e:
                    print ("/gazebo/pause_physics service call failed")
                #print("Pause done")
                
        def unpausePhysics(self):
                #print("Unpause called")
                rospy.wait_for_service('/gazebo/unpause_physics')
                try:
                    self.unpause()
                except (rospy.ServiceException) as e:
                    print ("/gazebo/unpause_physics service call failed")
                #print("Unpause done")
                    
        def resetSimulation(self):
                #print("Reset called")
                rospy.wait_for_service('/gazebo/reset_simulation')
                try:
                    self.reset_proxy()
                except (rospy.ServiceException) as e:
                    print ("/gazebo/reset_simulation service call failed")
                #print("Reset done")
    
        def _close(self):
                tmp = os.popen("ps -Af").read()
                gzclient_count = tmp.count('gzclient')
                gzserver_count = tmp.count('gzserver')
                roslaunch_count = tmp.count('roslaunch')

                if gzclient_count > 0:
                    os.system("killall -9 gzclient")
                if gzserver_count > 0:
                    os.system("killall -9 gzserver")
                if roslaunch_count > 0:
                    os.system("killall -9 roslaunch")

                if (gzclient_count or gzserver_count or roslaunch_count):
                    os.wait()
