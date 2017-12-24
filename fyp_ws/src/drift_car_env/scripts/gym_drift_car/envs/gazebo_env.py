import gym
from gym import error, spaces, utils
from gym.utils import seeding

import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu
from gazebo_msgs.msg import ModelStates

import numpy as np

import os
import signal
import subprocess
from os import path

class GazeboEnv(gym.Env):
        metadata = {'render.modes': ['human']}
        def __init__(self):
                subprocess.Popen("roscore")
                print ("Roscore launched!")
                
                rospy.init_node('gym', anonymous=True)
                
                subprocess.Popen(["roslaunch", "drift_car_gazebo", "drift_car.launch"])
                subprocess.Popen(["roslaunch", "drift_car_gazebo_control", "drift_car_control.launch "])
                
                print ("Gazebo launched!")      
                
                self.gzclient_pid = 0
                self.throtle1 = rospy.Publisher('/drift_car/joint1_position_controller/command', Float64, queue_size = 1)
                self.throtle2 = rospy.Publisher('/drift_car/joint2_position_controller/command', Float64, queue_size = 1)
                self.steer1 = rospy.Publisher('/drift_car/joint3_position_controller/command', Float64, queue_size = 1)
                self.steer2 = rospy.Publisher('/drift_car/joint4_position_controller/command', Float64, queue_size = 1)
        
                self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
                self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
                self.reset_proxy = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
                
                #TODO check how many actions
                self.action_space = spaces.Discrete(7)                             
                self.reward_range = (-np.inf, np.inf)
                self._seed()
                
                self.previous_action = -1
                self.radius = 1
                
                # Window for potential based reward.
                self.angle_reward_window = 0.1
                
        def _seed(self, seed=None):
                self.np_random, seed = seeding.np_random(seed)
                return [seed] 
                
        def _step(self, action):
                rospy.wait_for_service('/gazebo/unpause_physics')
                try:
                        self.unpause()
                except (rospy.ServiceException) as e:
                        print ("/gazebo/unpause_physics service call failed")


                #TODO can look into mirroring joints to make sure the wheels spin and turn tgt                
                #self.throtle1.publish(action.throttle)
		#self.throtle2.publish(action.throttle)
        
                degreeMappings = [65, 75, 85, 90, 95, 105, 115]
                radianMappings = [-0.436, -0.261799, -0.0872665, 0, 0.0872665, 0.261799, 0.436]              
                if(action < 7):
                        self.steer1.publish(radianMappings[action])
		        self.steer2.publish(radianMappings[action])                
                else:
                        return
                
                imuData = None
                posData = None
                while imuData is None:
                        try:
                                imuData = rospy.wait_for_message('/drift_car/imu_data', Imu, timeout=5)
                        except:
                                pass
                                
                while posData is None:
                        try:
                                posData = rospy.wait_for_message('/gazebo/model_states', ModelStates, timeout=5)
                                posData = posData.pose[1].position
                                posData.pose[1].orientation.w = abs(posData.pose[1].orientation.w)
                        except:
                                pass
                
                rospy.wait_for_service('/gazebo/pause_physics')
                try:
                        self.pause()
                except (rospy.ServiceException) as e:
                        print ("/gazebo/pause_physics service call failed")
                
                state = {"IMU": imuData, "pose": posData}
                reward = getReward(action, posData)
                done = false
              
                self.previous_imu = imuData
                self.previous_pos = posData     
                self.previous_action = action
                return state, reward, done, {}
                
        def getReward(self, action, posData):
                reward = 0.0
                
                largeActionDeltaPenalty = -100
                actionDelta = self.previous_action - action
                actionDeltaPenalty = (actionDelta ** 2) * largeActionDeltaPenalty
                
                # Calculate the potential reward based on polar angle difference.
                prev_angle = self.previous_pos.pose[1].orientation.w
                curr_angle = posData.pose[1].orientation.w
                if curr_angle > prev_angle or abs(prev_angle - curr_angle) <= self.angle_reward_window:
                        angle_potential_reward = 1
                else: 
                        angle_potential_reward = -1
                        
                # Calculate the potential reward based on circular path.
                x = posData.pose[1].position.x
                y = posData.pose[1].position.y
                deviationPenalty = -(abs((self.radius ** 2) - (x ** 2 + y ** 2)))
                
                
                reward = actionDeltaPenalty + angle_potential_reward + deviationPenalty
                return reward
                
        def _reset(self):
                rospy.wait_for_service('/gazebo/reset_simulation')
                try:
                    self.reset_proxy()
                except (rospy.ServiceException) as e:
                    print ("/gazebo/reset_simulation service call failed")

                rospy.wait_for_service('/gazebo/unpause_physics')
                try:
                    self.unpause()
                except (rospy.ServiceException) as e:
                    print ("/gazebo/unpause_physics service call failed")

                imuData = None
                posData = None
                while imuData is None:
                        try:
                                imuData = rospy.wait_for_message('/drift_car/imu_data', Imu, timeout=5)
                        except:
                                pass
                                
                while posData is None:
                        try:
                                posData = rospy.wait_for_message('/gazebo/model_states', ModelStates, timeout=5)
                                posData = posData.pose[1].position
                        except:
                                pass

                rospy.wait_for_service('/gazebo/pause_physics')
                try:
                    self.pause()
                except (rospy.ServiceException) as e:
                    print ("/gazebo/pause_physics service call failed")

                #TODO Figure out what state to send
                state = self.discretize_observation(data,5)
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
    
        def _close(self):
                tmp = os.popen("ps -Af").read()
                gzclient_count = tmp.count('gzclient')
                gzserver_count = tmp.count('gzserver')
                roscore_count = tmp.count('roscore')
                rosmaster_count = tmp.count('rosmaster')

                if gzclient_count > 0:
                    os.system("killall -9 gzclient")
                if gzserver_count > 0:
                    os.system("killall -9 gzserver")
                if rosmaster_count > 0:
                    os.system("killall -9 rosmaster")
                if roscore_count > 0:
                    os.system("killall -9 roscore")

                if (gzclient_count or gzserver_count or roscore_count or rosmaster_count >0):
                    os.wait()

