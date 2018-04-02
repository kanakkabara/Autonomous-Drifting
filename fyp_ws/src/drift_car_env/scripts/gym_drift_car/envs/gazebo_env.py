#!/usr/bin/env python
import gym
from gym import error, spaces, utils
from gym.utils import seeding

import rospy
import tf
from std_msgs.msg import Float64
from std_srvs.srv import Empty
from sensor_msgs.msg import Imu
from gazebo_msgs.msg import ModelStates, ModelState
from gazebo_msgs.srv import SetModelState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3Stamped, TransformStamped

import numpy as np

import os
import math
import signal
import subprocess
import time
from os import path

class GazeboEnv(gym.Env):
        metadata = {'render.modes': ['human']}
        # Default state: x, y, i, j, k, w, xdot, ydot, thetadot, s, xdotbodyframe, ydotbodyframe. 
        DEFAULT_STATE_INFO = {'state_size': 12, 'include_theta': True, 'include_position': True,
                'include_tangential_speed': True, 'include_body_frame_velocity': True}

        def __init__(self, continuous=False, state_info=DEFAULT_STATE_INFO, four_wheel_drive=False):
                rospy.init_node('gazebo_drift_car_gym')
                
                self.gazeboProcess = subprocess.Popen(["roslaunch", "drift_car_gazebo", "drift_car.launch"])
                time.sleep(10)
                # self.estimateProcess = subprocess.Popen(["roslaunch", "drift_car_gazebo", "gazebo_ekf.launch"])
                self.controlProcess = subprocess.Popen(["roslaunch", "drift_car_gazebo_control", "drift_car_control.launch"])
                time.sleep(5)
                                
                print ("Gazebo launched!")      
                
                self.gzclient_pid = 0
                self.four_wheel_drive = four_wheel_drive
                self.throtle1 = rospy.Publisher('/drift_car/left_rear_axle_controller/command', Float64, queue_size = 1)
                self.throtle2 = rospy.Publisher('/drift_car/right_rear_axle_controller/command', Float64, queue_size = 1)
                if four_wheel_drive:
                        self.throtle3 = rospy.Publisher('/drift_car/left_front_axle_controller/command', Float64, queue_size = 1)
                        self.throtle4 = rospy.Publisher('/drift_car/right_front_axle_controller/command', Float64, queue_size = 1)
                self.steer1 = rospy.Publisher('/drift_car/left_steering_joint_controller/command', Float64, queue_size = 1)
                self.steer2 = rospy.Publisher('/drift_car/right_steering_joint_controller/command', Float64, queue_size = 1)
        
                rospy.Subscriber('/drift_car/odom', Odometry, self.tfPublisher)
                # self.tl = tf.TransformListener()
                self.tb = tf.TransformBroadcaster()
                
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
                        minDegrees = 45
                        maxDegrees = 135
                        self.degreeMappings = range(minDegrees, maxDegrees+1, 10)
                        self.radianMappings = [math.radians(x-90) for x in self.degreeMappings]
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
                self.throttle = 1770      
                self.maxDeviationFromCenter = 4
                
        def _seed(self, seed=None):
                self.np_random, seed = seeding.np_random(seed)
                return [seed] 
        
        def applyThrottle(self, throtle):
                self.throtle1.publish(throtle)
                self.throtle2.publish(throtle)
                if self.four_wheel_drive:
                        self.throtle3.publish(throtle)
                        self.throtle4.publish(throtle)

        def applySteering(self, steering):
                self.steer1.publish(steering)
                self.steer2.publish(steering)

        def _step(self, action):
                #TODO can look into mirroring joints to make sure the wheels spin and turn tgt                
                
                self.unpausePhysics()

                if isinstance(action, tuple):
                        self.applyThrottle(action[0])
                        action = action[1]
                else:
                        self.applyThrottle(self.throttle)
                
                if self.continuous:
                        self.applySteering(action)
                else:
                        self.applySteering(self.radianMappings[action])

                posData = self.getPosData()
                #imuData = self.getIMUData()                

                self.pausePhysics()

                state = self.getState(posData)
                state = np.array([state[2], state[3], state[4]])
                reward = self.getRewardExponential(state[-4:])
                done = self.isDone(posData)
              
                #self.previous_imu = imuData
                self.previous_pos = posData     
                self.previous_action = action
                return state, reward, done, {}

        def getState(self, posData):
                # odomEstimate = self.getOdomEstimate()
                # velxEstimate = odomEstimate.twist.twist.linear.x
                # velyEstimate = odomEstimate.twist.twist.linear.y

                pose = posData.pose[1]
                position = pose.position
                orientation = pose.orientation

                t = tf.TransformerROS(True, rospy.Duration(10.0))
                m = TransformStamped()
                m.header.frame_id = 'world'
                m.child_frame_id = 'base_link'
                m.transform.translation.x = position.x
                m.transform.translation.y = position.y
                m.transform.translation.z = position.z
                m.transform.rotation.x = orientation.x
                m.transform.rotation.y = orientation.y
                m.transform.rotation.z = orientation.z
                m.transform.rotation.w = orientation.w
                t.setTransform(m)

                velx = posData.twist[1].linear.x
                vely = posData.twist[1].linear.y
                velz = posData.twist[1].linear.z

                # Transform to body frame velocities
                velVector = Vector3Stamped()
                velVector.vector.x = velx
                velVector.vector.y = vely
                velVector.vector.z = velz
                velVector.header.frame_id = "world"
                # velVectorTransformed = self.tl.transformVector3("base_link", velVector)
                velVectorTransformed = t.transformVector3("base_link", velVector)
                velxBody = velVectorTransformed.vector.x
                velyBody = velVectorTransformed.vector.y
                velzBody = velVectorTransformed.vector.z

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

                if 'include_body_frame_velocity' in self.state_info and self.state_info['include_body_frame_velocity']:
                    state += [velxBody, velyBody]

                return np.array(state)

        def getRewardExponential(self, state):
                # desiredTangentialSpeed = 5          # Tangential speed with respect to car body.
                # desiredNormalSpeed  = 0           # Perfect circular motion
                desiredAngularVel = -3.5
		desiredForwardVel = 0.5		
		desiredSideVel = 2 

                # velx = posData.twist[1].linear.x
                # vely = posData.twist[1].linear.y
                # carTangentialSpeed = math.sqrt(velx ** 2 + vely ** 2)
                # carAngularVel = posData.twist[1].angular.z
		carAngularVel = state[0]
		carForwardVel = state[1]
		carSideVel = state[2]

                sigma = 5
                deviationMagnitude = (carSideVel - desiredSideVel)**2 + \
                                (carForwardVel - desiredForwardVel)**2 + \
                                (carAngularVel - desiredAngularVel)**2
                
                #deviationMagnitude = (desiredTangentialSpeed - carTangentialSpeed)**2 + \
                #                (carAngularVel - desiredAngularVel)**2

                return 1 - math.exp(-deviationMagnitude/(2 * sigma**2))
        
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
                    nullPosition.pose.position.z = 0.03
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
                        except Exception as e:
                                failureCount += 1
                                if failureCount % 10 == 0:
                                        self.handleGazeboFailure()          
                                print(e)
                                pass
                #print("Fetched Pos Data")
                return posData
        
        def getOdomEstimate(self):
                #print("Fetching Pos Data")
                failureCount = 0
                odomData = None        
                while odomData is None:
                        try:
                                odomData = rospy.wait_for_message('/odometry/filtered', Odometry, timeout=1)
                        except Exception as e:
                                failureCount += 1
                                if failureCount % 10 == 0:
                                        self.handleGazeboFailure()          
                                print(e)
                                pass
                #print("Fetched Pos Data")
                return odomData

        def tfPublisher(self, data):
                pose = data.pose.pose
                position = pose.position
                orientation = pose.orientation

                self.tb.sendTransform((position.x, position.y, position.z), 
                                        (orientation.x, orientation.y, orientation.z, orientation.w), 
                                        rospy.Time.now(), 
                                        "base_link", 
                                        "world")

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
