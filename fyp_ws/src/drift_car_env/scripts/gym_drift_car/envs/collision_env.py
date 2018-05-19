#!/usr/bin/env python
import gym
from gym import error, spaces, utils
from gym.utils import seeding

import rospy
from std_msgs.msg import Float64
from std_srvs.srv import Empty
from sensor_msgs.msg import Imu, Image
from gazebo_msgs.msg import ModelStates, ModelState, ContactsState
from gazebo_msgs.srv import SetModelState

from cv_bridge import CvBridge, CvBridgeError

import numpy as np

import os
import math
import signal
import subprocess
import time
from os import path

class CollisionEnv(gym.Env):
        metadata = {'render.modes': ['human']}
        def __init__(self, continuous=False):
                rospy.init_node('gazebo_collision_car_gym')
                
                self.gazeboProcess = subprocess.Popen(["roslaunch", "drift_car_gazebo", "avoidance.launch"])
                time.sleep(10)
                self.controlProcess = subprocess.Popen(["roslaunch", "drift_car_gazebo_control", "drift_car_control.launch"])
                time.sleep(5)
                                
                print ("Gazebo launched!")      
                
                self.gzclient_pid = 0
                self.throtle1 = rospy.Publisher('/drift_car/left_rear_axle_controller/command', Float64, queue_size = 1)
                self.throtle2 = rospy.Publisher('/drift_car/right_rear_axle_controller/command', Float64, queue_size = 1)
                self.steer1 = rospy.Publisher('/drift_car/left_steering_joint_controller/command', Float64, queue_size = 1)
                self.steer2 = rospy.Publisher('/drift_car/right_steering_joint_controller/command', Float64, queue_size = 1)
        
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
                        carDegs = 25
                        minDegrees = 90 - carDegs
                        maxDegrees = 90 + carDegs
                        self.degreeMappings = range(minDegrees, maxDegrees+1, 10)
                        self.radianMappings = [math.radians(x-90) for x in self.degreeMappings]
                        self.action_space = spaces.Discrete(len(self.degreeMappings))
                
                self.bridge = CvBridge()

                #State related
                self.image_size = 84 * 84 * 3
                high = np.ones(self.image_size) * 255
                self.observation_space = spaces.Box(np.zeros(self.image_size), high)   
                
                self._seed()
                  
                # Learning Parameters
                self.radius = 3
                self.throttle = 1550      
                self.maxDeviationFromCenter = 10
                
        def _seed(self, seed=None):
                self.np_random, seed = seeding.np_random(seed)
                return [seed] 
        
        def applyThrottle(self, throtle):
                self.throtle1.publish(throtle)
                self.throtle2.publish(throtle)

        def applySteering(self, steering):
                self.steer1.publish(steering)
                self.steer2.publish(steering)

        def _step(self, action):
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

                state, collisionCount, posData = self.getState()
                self.pausePhysics()

                reward = 1
                done = self.isDone(collisionCount, posData)
              
                return state, reward, done, {"collisionCount": collisionCount}

        def getState(self):
                image = self.getImageData()
                posData = self.getPosData()
                collisionCount = self.getCollisionData()

                return np.array(image), collisionCount, posData

        def isDone(self, collisionCount, posData):       
                #Done is true if the car ventures too far from the center of the circular drift
                x = posData.pose[-1].position.x
                y = posData.pose[-1].position.y
                return (self.maxDeviationFromCenter <= ((x ** 2 + y ** 2) ** 0.5)) or (collisionCount > 0)
                
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
                state = self.getState()
                self.pausePhysics()

                return state
        
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
                        
                self.gazeboProcess = subprocess.Popen(["roslaunch", "drift_car_gazebo", "avoidance.launch"])
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

        def getCollisionData(self):
                #print("Fetching Pos Data")
                failureCount = 0
                collisionCount = 0
                left_front = None
                right_front = None
                left_rear = None
                right_rear = None
                chassis = None         
                while left_front is None or right_front is None or left_rear is None or right_rear is None:
                        try:
                                left_front = rospy.wait_for_message('/drift_car/collision/left_front', ContactsState, timeout=1)
                                right_front = rospy.wait_for_message('/drift_car/collision/right_front', ContactsState, timeout=1)
                                left_rear = rospy.wait_for_message('/drift_car/collision/left_rear', ContactsState, timeout=1)
                                right_rear = rospy.wait_for_message('/drift_car/collision/right_rear', ContactsState, timeout=1)
                                chassis = rospy.wait_for_message('/drift_car/collision/chassis', ContactsState, timeout=1)
                        except Exception as e:
                                failureCount += 1
                                if failureCount % 10 == 0:
                                        self.handleGazeboFailure()          
                                print(e)
                                pass
                
                return self.countCollisions(left_front) + self.countCollisions(right_front) + self.countCollisions(left_rear) + self.countCollisions(right_rear) + self.countCollisions(chassis)

        def countCollisions(self, collisionMsg):
                collisionCount = 0
                for contact in collisionMsg.states:
                        if not contact.collision2_name == "ground_plane::link::collision":
                                collisionCount += 1
                return collisionCount

        def getImageData(self):
            #print("Fetching Pos Data")
            failureCount = 0
            imageData = None        
            while imageData is None:
                    try:
                        imageData = rospy.wait_for_message('/drift_car/camera/image_raw', Image, timeout=1)
                    except Exception as e:
                            failureCount += 1
                            if failureCount % 10 == 0:
                                    self.handleGazeboFailure()          
                            print(e)
                            pass
            #print("Fetched Pos Data")
            try:
                cv_image = self.bridge.imgmsg_to_cv2(imageData, "rgb8")
            except CvBridgeError as e:
                print(e)
            return np.reshape(cv_image,[self.image_size])

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
