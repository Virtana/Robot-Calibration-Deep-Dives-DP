# Robot-Calibration-Deep-Dives-DP


A [ROS](https://www.ros.org/) package to simulate a simple 2D, 2-joint robotic arm with joint angle sensors, and calibrate the sensors using [ceres-solver](http://ceres-solver.org/index.html) library. [Install ROS here.](http://wiki.ros.org/ROS/Installation) The distribution of ROS used is [Melodic Morenia.](http://wiki.ros.org/melodic)

Required Libraries:
 * [ceres-solver](http://ceres-solver.org/installation.html)
 * [boost](https://www.boost.org/)
 * [yaml-cpp](https://github.com/jbeder/yaml-cpp)

## Installation 

Clone the repository into the src folder of a [catkin workspace.](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)
Enter the following commands to navigate back to the workspace directory and make the package using catkin:
```bash
cd ..
catkin_make
```

## Usage
Enter the following command to visualise the robot and write simulated data to file:
  ```bash 
  roslaunch robot_calibration joint_states_update.launch 
  ``` 

Wait until robot arm stops moving, and then terminate.

Enter the following command to start roscore, this is necessary for ROS nodes to run:
```bash
roscore
```

Open a new terminal and navigate to <your_catkin_workspace>/src/robot_calibration/sensor_data to obtain the name of the file which was generated with saved data, eg. sensor_data_1596547394.yaml. Enter the following command to perform calibration:
```bash
rosrun robot_calibration calibrator _filename:=your_filename
```
eg:
```bash
rosrun robot_calibration calibrator _filename:=sensor_data_1596547394.yaml
```
Observe terminal output to see results.

Offsets in each sensor can be changed by altering the values of the "theta_1_offset" and "theta_1_offset" parameters in robot_calibration/launch/joint_states_update.launch. 

Other config options, namely frequency of joint updates and number of joint updates, can be altered by changing the values in robot_calibration/config/2d_state_update_parameters.yaml.
