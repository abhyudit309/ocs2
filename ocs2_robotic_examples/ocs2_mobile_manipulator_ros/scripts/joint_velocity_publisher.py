#!/usr/bin/env python3

import time
import rospy
import tf
import numpy as np

from ocs2_msgs.msg import mpc_flattened_controller, mpc_observation
from kortex_driver.msg import Base_JointSpeeds, JointSpeed

class JointVelocityPublisher:

	def __init__(self):
		self.joint_vel_publisher = rospy.Publisher('/my_gen3/in/joint_velocity', Base_JointSpeeds, queue_size=10)

	def callback(self, data):
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
		#joint_vel = []
		#joint_vel = []
		dof = 7
		for i in range(dof, 2*dof):
			jvel = JointSpeed(joint_identifier=i-dof, value=data.state.value[i], duration=0)
			base_jvel = Base_JointSpeeds(joint_speeds=[jvel], duration=0)
			self.joint_vel_publisher.publish(base_jvel)
			#joint_vel.append(data.state.value[i])
			#print(data.state.value[i])
		#print(joint_vel)


	def subscribe(self):
		# rospy.Subscriber("/mobile_manipulator_mpc_policy", mpc_flattened_controller, callback)
		rospy.Subscriber("/mobile_manipulator_mpc_observation", mpc_observation, self.callback)
		rospy.spin()

def main():
    rospy.init_node('joint_velocity_publisher', anonymous=True)
    jvp = JointVelocityPublisher()
    jvp.subscribe()

if __name__ == '__main__':
	main()
	# try:
	# 	joint_vel_publisher()
	# except rospy.ROSInterruptException:
	# 	pass