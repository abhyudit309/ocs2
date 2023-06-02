#!/usr/bin/env python3

import time
import rospy
import tf
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from ocs2_msgs.msg import mpc_target_trajectories, mpc_state, mpc_input, mpc_observation

def pose_publisher():
	pub = rospy.Publisher('/mobile_manipulator_mpc_target', mpc_target_trajectories, queue_size=10)
	rospy.init_node('talker', anonymous=True)
	t_init = rospy.get_time()
	rate = rospy.Rate(0.25)

	### various poses for the arm ###

	retract1_angles = [180, 0, 90]
	retract1_xyz = [0.12954, 0.0, 0.20857]

	retract2_angles = [180, 0, 180]
	retract2_xyz = [0.0, 0.12954, 0.20857]

	retract3_angles = [180, 0, 0]
	retract3_xyz = [0.0, -0.12954, 0.20857]

	home_angles = [90, 0, 90]
	home_xyz = [0.5769, 0.0, 0.4341]

	retract1_quat = quaternion_from_euler(retract1_angles[0]*np.pi/180.0, retract1_angles[1]*np.pi/180.0, retract1_angles[2]*np.pi/180.0).tolist()
	retract1_pose = retract1_xyz + retract1_quat

	retract2_quat = quaternion_from_euler(retract2_angles[0]*np.pi/180.0, retract2_angles[1]*np.pi/180.0, retract2_angles[2]*np.pi/180.0).tolist()
	retract2_pose = retract2_xyz + retract2_quat

	retract3_quat = quaternion_from_euler(retract3_angles[0]*np.pi/180.0, retract3_angles[1]*np.pi/180.0, retract3_angles[2]*np.pi/180.0).tolist()
	retract3_pose = retract3_xyz + retract3_quat

	home_quat = quaternion_from_euler(home_angles[0]*np.pi/180.0, home_angles[1]*np.pi/180.0, home_angles[2]*np.pi/180.0).tolist()
	home_pose = home_xyz + home_quat

	#poses = [retract1_pose, home_pose]
	# poses = [home_pose]
	poses = [retract2_pose]

	### Now publish poses ###

	input_traj = [0, 0, 0, 0, 0, 0, 0]
	mpc_input_traj = [mpc_input(input_traj)]

	for i in range(len(poses)):
		state = [mpc_state(poses[i])]
		rate.sleep()
		current_time = rospy.get_time() - t_init
		target = mpc_target_trajectories([current_time], state, mpc_input_traj)
		print("Going to: ", poses[i][0:3])
		pub.publish(target)

if __name__ == '__main__':
	try:
		pose_publisher()
		rospy.spin()
	except rospy.ROSInterruptException:
		pass