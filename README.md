# OCS2 Toolbox

## Summary

OCS2 is a C++ toolbox tailored for Optimal Control for Switched Systems (OCS2). The toolbox provides an efficient implementation of the following algorithms:

* SLQ: Continuous-time domin DDP
* iLQR: Discrete-time domain DDP
* SQP: Multiple-shooting algorithm based on HPIPM
* PISOC: Path integral stochatic optimal control

OCS2 handles general path constraints through Augmented Lagrangian or relaxed barrier methods. To facilitate the application of OCS2 in robotic tasks, it provides the user with additional tools to set up the system dynamics (such as kinematic or dynamic models) and cost/constraints (such as self-collision avoidance and end-effector tracking) from a URDF model. The library also provides an automatic differentiation tool to calculate derivatives of the system dynamics, constraints, and cost. To facilitate its deployment on robotic platforms, the OCS2 provides tools for ROS interfaces. The toolbox’s efficient and numerically stable implementations in conjunction with its user-friendly interface have paved the way for employing it on numerous robotic applications with limited onboard computation power.

There are 2 main packages:

1. [`ocs2_robotic_examples/ocs2_mobile_manipulator`](ocs2_robotic_examples/ocs2_mobile_manipulator): This provides the library with the robot-specific MPC implementation.
2. [`ocs2_robotic_examples/ocs2_mobile_manipulator_ros`](ocs2_robotic_examples/ocs2_mobile_manipulator_ros): This wraps around the MPC implementation with ROS to define ROS nodes.

In our case, OCS2 is used for the control of:

* A fixed base 7 DOF Kinova Gen3 arm with a Robotiq 2F-85 gripper. The state consists of the 7 joint angles and 7 joint velocities. The control inputs are the 7 joint accelerations. The objective of the task is to track a 6 DOF end-effector pose. The joint position, velocity and acceleration limits are included in the constraint of the optimal control problem.

  ![manipulator](https://i.imgur.com/d6nmfcN.gif)

* The same arm on a mobile base with 4 caster wheels. Here the state consists of the 7 joint angles, 7 joint velocities, the 2D position and heading of the mobile base, and the heading angle and spin angle of each of the 4 wheels. The control inputs are the 7 joint accelerations, the 2D velocity and rate of change of heading of the base, and the rate of change of heading and spin angle of each wheel. The objective of the task is again to track a 6 DOF end-effector pose. The joint position, velocity, acceleration and wheel rotation and velocity limits are included in the constraint of the optimal control problem. There are additional constraints as each wheel rolls on the ground, which are specified in [`ocs2_robotic_examples/ocs2_mobile_manipulator/src/constraint/NoSlipConstraintCppAd.cpp`](ocs2_robotic_examples/ocs2_mobile_manipulator/src/constraint/NoSlipConstraintCppAd.cpp).

  ![mobile manipulator](https://i.imgur.com/f6akezz.gif)

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
    # Navigate to the src directory
    cd ~/ocs2_ws/src

    # Clone ocs2
    git clone https://github.com/abhyudit309/ocs2.git
    ```

4. For rigid multi-body dynamics library and self collision support, clone [Pinocchio](https://github.com/stack-of-tasks/pinocchio) and [HPP-FCL](https://github.com/humanoid-path-planner/hpp-fcl) into your workspace:

    ```bash
    # Navigate to the src directory
    cd ~/ocs2_ws/src
    
    # Clone pinocchio
    git clone --recurse-submodules https://github.com/leggedrobotics/pinocchio.git
    
    # Clone hpp-fcl
    git clone --recurse-submodules https://github.com/leggedrobotics/hpp-fcl.git
    ```

5. For various robotic assets, clone the following [repository](https://github.com/abhyudit309/ocs2_robotic_assets) into your workspace:

    ```bash
    # Navigate to the src directory
    cd ~/ocs2_ws/src
    
    # Clone ocs_robotic_assets
    git clone https://github.com/abhyudit309/ocs2_robotic_assets.git
    ```    

6. Build and run the unit tests:

    ```bash
    # Navigate to the src directory
    cd ~/ocs2_ws/src
    
    # Build
    catkin build ocs2

    # Source
    source ~/ocs2_ws/devel/setup.bash

    # run tests
    catkin run_tests ocs2
    ```
   
## Usage

All launch files are in [`ocs2_robotic_examples/ocs2_mobile_manipulator_ros/launch`](ocs2_robotic_examples/ocs2_mobile_manipulator_ros/launch). Firstly, navigate to the catkin workspace:

 ```bash
 cd ~/ocs2_ws/src
 ```

Then
* To run the 7 DOF Kinova Gen 3 arm with the Robotiq-2F-85 gripper:

   ```bash
   roslaunch ocs2_mobile_manipulator_ros manipulator_kinova_gen3_robotiq_2f_85.launch
   ```

* To run the 7 DOF Kinova Gen 3 arm with the Robotiq-2F-85 gripper on a unicycle base:

   ```bash
   roslaunch ocs2_mobile_manipulator_ros manipulator_kinova_gen3_robotiq_2f_85_platform_v1.launch
   ```

* To run the 7 DOF Kinova Gen 3 arm with the Robotiq-2F-85 gripper on a mobile base with 4 caster wheels:

   ```bash
   roslaunch ocs2_mobile_manipulator_ros manipulator_kinova_gen3_robotiq_2f_85_platform_v2.launch
   ```
