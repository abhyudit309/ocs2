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

1. Install all prerequisites for OCS2 by following the instructions [here](https://leggedrobotics.github.io/ocs2/installation.html).

2. Create a new catkin workspace:

   ```bash
   # Create the directories
   mkdir -p ~/ocs2_ws/src
   cd ~/ocs2_ws/
  
   # Initialize the catkin workspace
   catkin init
   catkin config --extend /opt/ros/noetic
   catkin config -DCMAKE_BUILD_TYPE=RelWithDebInfo
   ```

3. Clone this repository:

    ```bash
    # Navigate to the directory of src
    cd ~/ocs2_ws/src
    git clone https://github.com/abhyudit309/ocs2.git
    ```

4. Build and run the unit tests:

    ```bash
    # Build it
    catkin build ocs2

    # Source it
    source ~/ocs2_ws/devel/setup.bash

    # run tests
    catkin run_tests ocs2
    ```

## Dependencies

1. For rigid multi-body dynamics library and self collision support clone [Pinocchio](https://github.com/stack-of-tasks/pinocchio) and [HPP-FCL](https://github.com/humanoid-path-planner/hpp-fcl) into your workspace:

    ```bash
    # Navigate to the directory of src
    cd ~/ocs2_ws/src
    
    # Clone pinocchio
    git clone --recurse-submodules https://github.com/leggedrobotics/pinocchio.git
    
    # Clone hpp-fcl
    git clone --recurse-submodules https://github.com/leggedrobotics/hpp-fcl.git
    ```

2. For various robotic assets used clone the following [repository](https://github.com/abhyudit309/ocs2_robotic_assets) in your workspace:

    ```bash
    # Navigate to the directory of src
    cd ~/ocs2_ws/src
    
    # Clone ocs_robotic_assets
    git clone https://github.com/abhyudit309/ocs2_robotic_assets.git
    ```
   
## Usage

* To run the 7 DOF Kinova Gen 3 arm with the Robotiq-2F-85 gripper:

   ```bash
   roslaunch ocs2_mobile_manipulator_ros manipulator_kinova_gen3_robotiq_2f_85.launch
   ```

* To run the 7 DOF Kinova Gen 3 arm with the Robotiq-2F-85 gripper on a holonomic mobile base:

   ```bash
   roslaunch ocs2_mobile_manipulator_ros manipulator_kinova_gen3_robotiq_2f_85_platform_v1.launch
   ```

* To run the 7 DOF Kinova Gen 3 arm with the Robotiq-2F-85 gripper on a mobile base with 4 caster wheels:

   ```bash
   roslaunch ocs2_mobile_manipulator_ros manipulator_kinova_gen3_robotiq_2f_85_platform_v2.launch
   ```
