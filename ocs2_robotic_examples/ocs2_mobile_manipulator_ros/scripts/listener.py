#!/usr/bin/env python3

import time
import rospy
import tf
import numpy as np

from ocs2_msgs.msg import mpc_flattened_controller, mpc_observation

def callback(data):
	# data.stateTrajectory is a list of objects
	# dof = 7
	# no_of_states = len(data.stateTrajectory)
	# #states = np.zeros((no_of_states, dof))
	# for i in range(no_of_states):
	# 	f.write(str(data.timeTrajectory[i]) + " ")
	# 	for j in range(dof):
	# 		f.write(str(data.stateTrajectory[i].value[j]) + " ")
	# 	f.write('\n')
	# print(data.time)
	dof = 7
	if data.time > 0:
		if np.linalg.norm(np.array(data.input.value)) > 0.01:
			f1.write(str(data.time) + " ")
			for i in range(2*dof):
				f1.write(str(data.state.value[i]) + " ")
			f1.write('\n')

			f2.write(str(data.time) + " ")
			for i in range(dof):
				f2.write(str(data.input.value[i]) + " ")
			f2.write('\n')
		else:
			print("OVER!!!!!!!!!!!!!")

def listener():
	rospy.init_node('listener', anonymous=True)

	global f1, f2
	f1 = open('/home/abhyudit/ocs2_data/retract_joint_data.txt', 'w')
	f2 = open('/home/abhyudit/ocs2_data/retract_input_data.txt', 'w')

	# rospy.Subscriber("/mobile_manipulator_mpc_policy", mpc_flattened_controller, callback)
	rospy.Subscriber("/mobile_manipulator_mpc_observation", mpc_observation, callback)
	rospy.spin()

if __name__ == '__main__':
	try:
		listener()
	except rospy.ROSInterruptException:
		pass