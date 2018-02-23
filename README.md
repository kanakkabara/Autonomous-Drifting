# Autonomous-Drifting
Autonomous Drifting using Reinforcement Learning 

## Installation
1. sudo ./setup_env.sh
2. cd fyp_ws
3. catkin_make
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
Teleop Reward Tester | ``` rosrun drift_car teleopRewardTester.py ```
Double Dueling Deep Q-Network | ```rosrun drift_car main.py```

## PILCO
1. Install MATLAB, enabled with Robotics System Toolbox. 
2. Add src/drift_car/scripts/rl/modules and src/drift_car/scripts/rl/pilco to MATLAB path.
3. Start the bridge library with ```rosrun drift_car_env matlab_bridge.py```.
4. Initialize the MATLAB ROS node with ```rosinit```.
5. To train - ```drift_car_learn```. 
6. To apply learned controller - ```applyController```.