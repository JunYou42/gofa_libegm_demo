# gofa_libegm_samples

## Description
This ROS package provides some examples of controlling the robot GoFa via Externally Guided Motion (EGM). So far it includes following examples:
- a1_joint_trajectory_node
- a2_pose_trajectory_node
- c1_joint_controller_node
- c2_pose_controller_node

> The original sample codes can be found in [this issue](https://github.com/ros-industrial/abb_libegm/issues/18). 

> For more information regarding **abb_libegm**, click [here](https://github.com/ros-industrial/abb_libegm).


## Prerequisite
This package is tested with:

- [ ] Ubuntu 18.04 and ROS Melodic
- [ ] Ubuntu 20.04 and ROS Noetic

and built with [catkin_tools](https://catkin-tools.readthedocs.io/en/latest/index.html).


## Usage

### 1. Robotstuidio and ABB controller setup
a. access the controller in Robotstudio
b. change RAPID file
- for joint space, use main_t1() as the main function
![main_t1](image.jpg)
- for cartesain space, switch to main_t2()  
 
### 2. ROS package usage
a. Clone the repoistary to your local computer and build the package
```
mkdir catkin_ws/src
cd catkin_ws/src
git clone xxx
cd ..
catkin build
source devel/setup.bash
```

b. Run package
- open a new terminal and run
``` 
roscore
```
- open a new terminal and run 
```
rosrun gofa_libegm_samples gofa_libegm_samples_a1_joint_trajectory_node 
```
to send command of multiple points in joint space at once.
Or
```
rosrun gofa_libegm_samples gofa_libegm_samples_a2_pose_trajectory_node 
```
to send command of multiple points in cartesian space at once. 
Or
```
rosrun gofa_libegm_samples gofa_libegm_samples_c1_joint_controller_node 
```
to send command of one point in joint space. 
Or
```
rosrun gofa_libegm_samples gofa_libegm_samples_c2_pose_controller_node 
```
to send command of one point in cartesian space. 



> Note: check the RAPID file in the controller to make sure that for each sample the corresponding RAPID file is running.




