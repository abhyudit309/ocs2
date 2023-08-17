# OCS2 Toolbox

## Summary
OCS2 is a C++ toolbox tailored for Optimal Control for Switched Systems (OCS2). The toolbox provides an efficient implementation of the following algorithms

* SLQ: Continuous-time domin DDP
* iLQR: Discrete-time domain DDP
* SQP: Multiple-shooting algorithm based on HPIPM
* PISOC: Path integral stochatic optimal control

Here it is used for the control of a:

1. Fixed 7 DOF Kinova Gen3 arm with a Robotiq 2F-85 gripper

![manipulator](https://i.imgur.com/d6nmfcN.gif)

2. The same arm on a base with 4 caster wheels.

![mobile manipulator](https://i.imgur.com/f6akezz.gif)

OCS2 handles general path constraints through Augmented Lagrangian or relaxed barrier methods. To facilitate the application of OCS2 in robotic tasks, it provides the user with additional tools to set up the system dynamics (such as kinematic or dynamic models) and cost/constraints (such as self-collision avoidance and end-effector tracking) from a URDF model. The library also provides an automatic differentiation tool to calculate derivatives of the system dynamics, constraints, and cost. To facilitate its deployment on robotic platforms, the OCS2 provides tools for ROS interfaces. The toolbox’s efficient and numerically stable implementations in conjunction with its user-friendly interface have paved the way for employing it on numerous robotic applications with limited onboard computation power.

For more information refer to the project's [Documentation Page](https://leggedrobotics.github.io/ocs2/) 

## Setup

* Install all prerequisites for OCS2 by following the instructions [here](https://leggedrobotics.github.io/ocs2/installation.html)
* Create a new catkin workspace
  ```bash
  # Create the directories
  # Do not forget to change <...> parts
  mkdir -p <directory_to_ws>/<catkin_ws_name>/src
  cd <directory_to_ws>/<catkin_ws_name>/
  
  # Initialize the catkin workspace
  catkin init
  catkin config --extend /opt/ros/noetic
  catkin config -DCMAKE_BUILD_TYPE=RelWithDebInfo
  ```
* Clone this repository
