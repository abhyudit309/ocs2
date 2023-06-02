#!/usr/bin/env python3

import time
import rospy
import tf
import numpy as np

from ocs2_msgs.msg import mpc_flattened_controller, mpc_observation
from kortex_driver.msg import Base_JointSpeeds, JointSpeed
from sensor_msgs.msg import JointState

def joint_state_publisher():
	rospy.init_node('joint_velocity_publisher', anonymous=True)
	publisher = rospy.Publisher('/my_gen3/joint_states', JointState, queue_size=10)

	dof = 7
	js = JointState()
	js.name = [" ", " ", " ", " ", " ", " ", " ", " ", " ", " ", " ", " ", " "]
	js.position = [4.917, -0.35, 3.0, -3.14, 0, -0.87, 1.57, 0.0035, 0.0035, -0.0035, 0.0035, 0.0035, -0.0035]
	js.velocity = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
	js.effort = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

	while True:
		publisher.publish(js)
    
if __name__ == '__main__':
    try:
        joint_state_publisher()
    except rospy.ROSInterruptException:
        pass
	# try:
	# 	joint_vel_publisher()
	# except rospy.ROSInterruptException:
	# 	pass