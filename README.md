# Autonomous-Drifting
Autonomous Drifting using Reinforcement Learning 

## Installation
1. cd fyp_ws
2. catkin_make
3. . devel/setup.bash (add this to the .bashrc file so that you dont have to run it everytime, [need to add full path to file])
4. cd src/drift_car_env/scripts/
5. sudo pip install -e .

## Commands
To run | Command
--- | --- 
ROS Core | ``` roscore ```
Gazebo Simulator | ``` roslaunch drift_car_gazebo drift_car.launch ```
Controller | ``` roslaunch drift_car_gazebo_control drift_car_control.launch ```
Keyboard Teleop | ``` rosrun drift_car teleop.py ```
RL Agent | ```rosrun drift_car DriftingCarRL.py```
