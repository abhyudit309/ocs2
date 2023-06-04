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
	state = data.state.value
	inp = data.input.value
	a = 0.055;
	l = 0.11;
	d = 0.11;
	theta = state[2];
	p1 = state[3];
	p2 = state[4];
	q1 = state[5];
	q2 = state[6];
	r1 = state[7];
	r2 = state[8];
	s1 = state[9];
	s2 = state[10];
	x_dot = inp[0];
	y_dot = inp[1];
	theta_dot = inp[2];
	p1_dot = inp[3];
	p2_dot = inp[4];
	q1_dot = inp[5];
	q2_dot = inp[6];
	r1_dot = inp[7];
	r2_dot = inp[8];
	s1_dot = inp[9];
	s2_dot = inp[10];   

	c1 = x_dot + l*theta_dot*np.cos(theta) - d*theta_dot*np.sin(theta) - a*p2_dot*np.sin(p1 + theta);
	c2 = y_dot + l*theta_dot*np.sin(theta) + d*theta_dot*np.cos(theta) + a*p2_dot*np.cos(p1 + theta);

	c3 = x_dot + l*theta_dot*np.cos(theta) + d*theta_dot*np.sin(theta) - a*q2_dot*np.sin(q1 + theta);
	c4 = y_dot + l*theta_dot*np.sin(theta) - d*theta_dot*np.cos(theta) + a*q2_dot*np.cos(q1 + theta);

	c5 = x_dot - l*theta_dot*np.cos(theta) - d*theta_dot*np.sin(theta) - a*r2_dot*np.sin(r1 + theta);
	c6 = y_dot - l*theta_dot*np.sin(theta) + d*theta_dot*np.cos(theta) + a*r2_dot*np.cos(r1 + theta);

	c7 = x_dot - l*theta_dot*np.cos(theta) + d*theta_dot*np.sin(theta) - a*s2_dot*np.sin(s1 + theta);
	c8 = y_dot - l*theta_dot*np.sin(theta) - d*theta_dot*np.cos(theta) + a*s2_dot*np.cos(s1 + theta);

	constraint = [c1, c2, c3, c4, c5, c6, c7, c8]
	#print(constraint)
	# if data.time > 0:
	# 	if np.linalg.norm(np.array(data.input.value)) > 0.01:
	# 		f1.write(str(data.time) + " ")
	# 		for i in range(2*dof):
	# 			f1.write(str(data.state.value[i]) + " ")
	# 		f1.write('\n')

	# 		f2.write(str(data.time) + " ")
	# 		for i in range(dof):
	# 			f2.write(str(data.input.value[i]) + " ")
	# 		f2.write('\n')
	# 	else:
	# 		print("OVER!!!!!!!!!!!!!")

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